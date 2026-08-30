# GPT-Pro consult #3 (2026-08-30): contact-force extraction + implementation red-team

Requested by user; submitted via ego-browser to the user's ChatGPT (Pro plan, reasoning
effort "Extra High"), conversation 6a93a3b5-b964-83e9-8c2d-dac4f0dcff31. Prompt: full
context of the SolverVBD.body_forces attribution failure (block-absent control reads
commanded 1.2 N + ~20 N penalty with no soft body present) plus a 9-item red-team list.
Full response verbatim below. Judge verification of the latch-timing claim: CONFIRMED
against frozen src/judgment_vbd.py (LIFT_END=1.80 grip-completion latch, receipts
damage_window=grip_completion_preload_end) -> paper SIII wording fixed same day.

---

Q1. Trustworthy per-pad contact force extraction

결론부터 말하면, SolverVBD.body_forces를 force sensor channel로 쓰면 안 됨. 현재 attribution probe의 block-absent failure가 정확히 그 사실을 검출한 것. body_forces는 rigid VBD solve 중 사용하는 working accumulator이고 body-particle contact 외에 rigid contact / joint constraint 계열 항과 같은 rigid solve 경로를 공유함. Control.joint_f도 rigid integration을 통해 상태와 constraint reaction에 들어가므로, block이 없어도 pad body row가 0일 이유가 없음. W2를 GEOMETRY_ONLY로 내린 판단이 맞음.

(a) 실제 soft-rigid force가 어디 있는가

이 Newton commit의 soft-rigid VBD path에는 persistent soft_contact_lambda나 per-contact impulse array가 없음. rigid_compliant_alm=True여도 이 부분은 달라지지 않음. Soft-rigid는 body-particle penalty contact이고, 각 solve evaluation에서 force를 계산한 뒤 particle/body accumulator로 더함. 반대로 body_body_contact_lambda, collect_rigid_contact_forces(), Contacts.rigid_contact_force는 rigid-rigid path임. Newton 자체 basic_conveyor_forces example도 VBD의 rigid force report에는 collect_rigid_contact_forces()를 명시적으로 호출함.

Pointer	의미	사용 판단
Contacts.soft_contact_count	active soft contact record count	사용
soft_contact_indices + soft_contact_barycentric	particle/edge/face soft feature 정의	full-surface에서는 필수
soft_contact_shape, soft_contact_body_pos, soft_contact_body_vel, soft_contact_normal	rigid-side geometry/kinematics	사용
SolverVBD.body_particle_contact_penalty_k	현재 per-contact penalty stiffness	사용
body_particle_contact_material_kd, ..._mu	mixed contact properties	사용
_compute_body_particle_contact_force()	실제 soft-rigid force law	핵심
_eval_body_particle_contact()	vertex contact evaluator	핵심
_eval_soft_ef_contact()	particle/edge/face unified evaluator	가장 중요
_harvest_vbd_body_particle_contact_forces_on_proxy_bodies_kernel	위 evaluator로 final soft-rigid wrench를 harvest하는 Newton 자체 kernel	권장 template
SolverVBD.body_forces	rigid solver scratch/aggregate	사용 금지
Contacts.force	generic extended force attribute	b74df53의 SolverVBD soft path에서 의존 금지

Full-surface record는 (p,-1,-1), (v0,v1,-1), (v0,v1,v2) 형태로 같은 soft_contact_indices 공간에 들어가며 barycentric weight로 contact point가 정의됨. 따라서 soft_contact_particle만 읽으면 edge/face contact를 누락함.

더 중요한 pointer가 vbd_coupling_kernels._harvest_vbd_body_particle_contact_forces_on_proxy_bodies_kernel임. Newton 개발자들이 VBD coupling feedback을 위해 이미 만든 canonical force harvester이고, 내부에서 _eval_soft_ef_contact()를 호출한 뒤 정확히

force_on_body = -force_on_particle
torque_on_body = (contact_point - body_COM) × force_on_body

로 wrench를 누적함. 즉 여러분이 필요한 pad reaction과 사실상 동일한 계산임.

Force law도 명확함. Soft-rigid contact는 대략

F
n
elastic
	​

=k
e
	​

δ

이고 closing motion에서 kd normal damping이 추가됨. Tangential은 friction_epsilon으로 regularize한 isotropic Coulomb law이며, friction cone의 normal load는 damping까지 포함한 total normal이 아니라 ke*penetration인 elastic normal load임. 양쪽 material이 모두 ke=1000, kd=1, mu=1이면 arithmetic/geometric mixing 후에도 정확히 1000 / 1 / 1임.

Contacts.force를 request해서 해결하는 것도 현재 pin에서는 권하지 않음. Extended-contact docs가 명시적으로 force를 populate하는 solver로 SolverMuJoCo, SolverXPBD를 언급하지만 SolverVBD는 해당 목록에 없음.

(b) 권장 cheap accumulation recipe

가장 작은 변경은 Newton의 private harvester kernel을 그대로 이용하는 것. 연구 artifact가 이미 Newton commit에 pin되어 있으므로 private API 사용도 재현성 관점에서는 허용 가능하다고 봄. 다만 commit 변경 시 깨질 수 있으므로 project-local wrapper로 격리하는 편이 좋음.

각 substep에서 collision_pipeline.collide() 직전의 body_q를 persistent body_q_pre buffer에 복사. 필요하면 particle pre-state도 별도 보관. solver.step() 후, ping-pong swap 전에 collector 실행.

body_local_to_pad를 body-count 길이 int array로 만들고 전부 -1, b_left→0, b_right→1로 설정. pad_wrench = wp.zeros(2, dtype=wp.spatial_vector).

_harvest_vbd_body_particle_contact_forces_on_proxy_bodies_kernel을 dim=contacts.soft_contact_max로 launch. 이름은 proxy용이지만 이 commit의 soft-contact branch는 실제로 mapping array로 대상 body를 선택하므로 pad 두 개만 mapping하면 됨. 입력은 post-step particle_q/body_q, pre-step state, body_particle_contact_penalty_k/kd/mu, 그리고 contacts.soft_contact_*.

wp.spatial_top(pad_wrench[p])가 tofu가 pad에 가한 world-frame net force. Pad의 outward unit normal을 body orientation으로 world frame에 변환해서 Fn = dot(Fpad,n_out), Ft_vec = Fpad - Fn*n_out, Ft=||Ft_vec||로 계산. Left/right sign convention은 현재 w2_attr_probe.py의 outward normal convention과 맞추면 됨.

Frame-average가 필요하면 force 자체 평균, impulse가 필요하면 J = Σ F_k Δt. VBD iteration별 force를 impulse처럼 합산하면 안 됨. Iterations는 nonlinear solve 과정이지 physical micro-timestep이 아님.

Launch 구조는 대략 다음 형태면 됨.

Python
Run
pad_wrench.zero_()

wp.launch(
    _harvest_vbd_body_particle_contact_forces_on_proxy_bodies_kernel,
    dim=contacts.soft_contact_max,
    inputs=[
        dt,
        body_local_to_pad,
        state_post.particle_q,
        particle_q_pre,          # or solver's matching pre-step buffer
        model.particle_radius,
        state_post.body_q,
        body_q_pre,
        state_post.body_qd,
        model.body_com,
        solver.friction_epsilon,
        solver.body_particle_contact_penalty_k,
        solver.body_particle_contact_material_kd,
        solver.body_particle_contact_material_mu,
        contacts.soft_contact_count,
        contacts.soft_contact_indices,
        contacts.soft_contact_barycentric,
        contacts.soft_contact_shape,
        contacts.soft_contact_body_pos,
        contacts.soft_contact_body_vel,
        contacts.soft_contact_normal,
        model.shape_margin,
        model.shape_body,
    ],
    outputs=[pad_wrench],
)

이것은 Newton 내부 coupling code와 같은 path임.

Per-contact 값까지 필요하면 이 kernel을 약간 복제해서 out_force_body[i], out_contact_point[i], out_pad_id[i]를 write하는 편이 좋음. 이후 CPU에서 stable order로 float64 sum하면 recorder 자체의 atomic_add reduction nondeterminism까지 제거 가능. Contact 수가 작으므로 비용도 미미함.

Nodal reconstruction은 차선책임. particle_vbd_kernels.accumulate_particle_body_contact_force_and_hessian()도 같은 _eval_*를 호출해서 particle forces에 더하지만, edge/face force를 barycentric으로 분산하고 다른 force terms와 같은 particle accumulator를 사용함. 따라서 최종 total nodal force에서 접촉 vertex만 골라 합하는 방식은 권장하지 않음. Dedicated contact contribution을 instrument하거나 직접 per-contact evaluator를 쓰는 편이 훨씬 깨끗함.

Trust test는 기존 ATTR probe를 그대로 확장하면 됨. Block absent에서는 soft-contact-only collector가 machine-zero 수준이어야 하고, static hold에서는 각 finger Fn≈F_cmd; 전체 tofu momentum balance에서는

F
L
pad→soft
	​

+F
R
pad→soft
	​

+mg≈
dt
dP
soft
	​

	​


가 성립해야 함. 특히 64 g block의 정적 hold라면 두 pad의 vector-summed vertical friction이 약 0.628 N을 지지해야 함. 이 검사가 joint_f와 독립적인 가장 강한 attribution falsifier 중 하나임.

(c) Tangential force의 주요 함정

현재 friction_epsilon=2e-4는 단위가 사실상 velocity threshold m/s임. Code가 eps_u = friction_epsilon * dt로 displacement threshold를 만들기 때문. 80 substeps에서 dt=0.2083 ms, 따라서 smoothing displacement는 불과 4.17e-8 m임. Regularized branch에서 작은 slip의 effective viscous slope는 대략

ϵ
v
	​

2μF
n
	​

	​


이므로 Fn=1 N에서 약 10,000 N·s/m, Fn=1.2 N에서 약 12,000 N·s/m임. 0.2 mm/s 이하 영역에서는 매우 강한 stick-like regularization임. 정확히 zero slip에서 persistent static-friction state가 존재하는 것은 아니지만, 아주 작은 creep velocity만으로 sub-cone load를 지지하게 됨.

따라서 |Ft|/Fn을 해석할 때도 denominator 정의 필요. Solver의 Coulomb cap과 비교하려면 Σ ke*δ가 맞고, sensor-like pad resultant를 원하면 damping을 포함한 net normal reaction을 쓰는 것이 맞음. 둘은 transient에서 다를 수 있음.

또한 full-surface contact에서는 contact record 수가 mesh/contact topology에 따라 변하고 ke가 N/m per contact record이므로 patch aggregate stiffness가 contact count에 의존함. Per-contact tangential force나 peak ratio는 mesh-invariant observable이 아님. Net pad wrench가 훨씬 안정적인 quantity임. 이 점은 force extraction보다 parameter sensitivity 문제에 가까움.

Q2. Ranked red-team

여기서는 “이 frozen simulator에서 이런 band가 나왔다”보다 한 단계 강한 physical grasp-feasibility phase diagram claim을 기준으로 순위를 매김. Frozen-model descriptive claim만 한다면 특히 #6과 #9의 위험도는 한 단계씩 내려감.

Rank	항목	Risk	판단 / 가장 싼 falsification
1	(6) damage = elastic strain proxy	HIGH	현재 closure가 damage branch에 상당히 의존함. 예를 들어 E7/E15의 closure row는 low-F slip + high-F damage 구조이고, E25 a=10도 F=2가 damage라서 이것을 intact로 바꾸면 closure가 달라질 수 있음. fracture/plasticity가 없으므로 "damage"보다는 "strain-threshold damage proxy"가 정확함. 가장 싼 검사는 저장된 strain field만으로 eps={0.10,0.15,0.20} × DVF={0.25%,0.5%,1%} offline relabel 후 P-B closure가 유지되는지 확인하는 것. 추가로 current master는 실제 latch 시작이 “post-lift”가 아니라 1.80 s, 즉 grip-completion/preload-end임. 함수명은 과거 이름이 남아 있음. Paper wording 정정 필요.
2	(1) substeps 40 chaotic, 80 stable	HIGH	80이 convergence point라는 증거가 아직 없음. Production mesh는 h=5 mm임. ν=.45, ρ=1000에서 stiffest E=25 kPa의 compressional wave speed는 약 9.74 m/s, 따라서 h/cp≈0.513 ms. dt40=0.417 ms≈0.81 h/cp, dt80=0.208 ms≈0.41 h/cp, dt160≈0.20 h/cp. VBD가 implicit라 hard CFL은 아니지만 40의 failure와 매우 일관된 temporal-resolution scale임. Cheap test: boundary sentinel 6–9 cells만 80/160; 추가로 한두 개를 (80, iterations=20)도 비교. Label 불변 + realized-a <2% + slip residual <0.25 mm + DVF <0.001 abs 정도면 강한 convergence evidence.
3	(9) ke/kd/mu physical identification	HIGH / MED-HIGH	내부적으로는 coherent하지만 물리 calibration은 별개. mu=1은 slip boundary를 직접 결정하고, ke=1e3 N/m/contact는 contact count에 따라 aggregate stiffness가 변함. 4 cm cube의 단순 EA/L scale은 E7/15/25에서 약 280/600/1000 N/m인데, interface에는 여러 1 kN/m records가 병렬로 생김. 즉 interface는 상당히 hard하고 mesh-dependent함. h=5 mm grid의 평균 node mass scale로 보면 kd=1은 single-record critical damping scale(~0.6 Ns/m)보다도 큼; transient strain에 영향 가능. Cheap test는 경계 cells에 mu={0.8,1.0,1.2}, ke={0.5,1,2} kN/m, kd={0.5,1,2}의 sparse one-at-a-time sweep. mu sensitivity가 가장 중요.
4	(2) friction_epsilon=2e-4	MED-HIGH	Quasi-static creep를 없애기 위해 선택했다는 사실 자체가 결과 sensitivity 가능성을 의미함. Code상 이 값은 low-speed creep law를 사실상 조절함. 다만 substep과 함께 eps_u=eps_v dt로 scale되므로 40→80에서 velocity threshold 자체는 변하지 않는다는 점은 좋음. Falsifier: slip/intact boundary만 eps={1e-4,2e-4,5e-4,1e-3}. 특히 1e-4와 2e-4가 같으면 epsilon→0 방향 convergence 주장 가능.
5	(3) GPU same-seed nondeterminism	MEDIUM	Legitimate GPU behavior일 가능성이 높지만 3-seed 2/3 rule이 이것을 완전히 해결하지는 않음. Newton b74에는 SolverVBD(... deterministic=wp.DeterministicMode.RUN_TO_RUN) 지원과 determinism test가 존재함. Full-surface edge/face contact emission 자체도 shared counter를 사용하므로 contact ordering이 민감한 구조임. 6개 boundary cell × 동일 seed 10회로 label entropy 측정 후, wp.config.deterministic=RUN_TO_RUN + SolverVBD deterministic + CollisionPipeline(deterministic=True)로 반복. Trajectory만 다르고 label이 10/10 동일이면 낮은 위험. 동일 seed에서 label flip이면 seed voting과 별개로 numerical robustness issue.
6	(5) a=30에서 ~0.1 s 후 grasp loss / free carriage	MEDIUM for axis, LOW for core closure	Failure exposure 자체는 유효함. 즉 "commanded level 30에서 plateau 도달 전에 failure"라는 outcome은 정당함. 반면 그 trial을 sustained realized 19.8 m/s² exposure라고 부르면 안 됨. Prereg도 top realized level이 free carriage calibration이며 “grasped ejects instantly”라고 명시함. failure-before-plateau로 표시하고 pre-loss palm acceleration/peak 또는 command level을 함께 보존하는 것이 맞음. E7/E15/E25 closure는 더 낮은 a에서 이미 발생하므로 P-B core verdict에는 거의 영향 없음.
7	(4) realized-acceleration axis	LOW-MEDIUM	기본 방향은 sound. 현재 medians 0.681,1.647,3.183,6.402,12.889,19.846은 monotone이며 paper가 realized axis를 쓰는 것도 적절함. 다만 측정 대상은 tofu COM이 아니라 palm/carriage apparatus acceleration이어야 하고, post-slip unloading 때문에 cell-specific acceleration이 outcome-dependent해지는 것을 피해야 함. commanded level = treatment, realized palm acceleration = calibrated physical axis로 둘 다 유지 권장. a=30은 위의 censoring 예외.
8	(8) 0.5 mm pad-relative fidelity exceedance	LOW	현재 evidence는 오히려 reassuring함. Judge sweep에서 18 pad rows = 9 receipts가 모두 F=2.0 N, a>=10, damage-labeled이고 range 0.51–0.61 mm; intact 최대는 0.276 mm임. W1 label은 pad geometry가 아니라 state/strain이고 W2는 F=1.2 sweep이므로 core conclusion 위협 거의 없음. 다만 향후 intact row에서 0.5 mm 초과 시 hard failure로 보는 현재 rule이 적절함.
9	(7) no-contact를 recorder dropout으로 센 spec 수정	LOW; 수정이 타당함	기존 rule이 category error였음. Full sweep에서 모든 pad-row가 최소 ~601 zero-record substeps를 가지며 approach 전/슬ip 후 physical separation과 일치함. 즉 original vg2==0은 애초에 만족 불가능. W1 state-log inference에 finite AND overflow-free를 적용한 channel-scoping은 타당함. 단 W2에는 expected-contact 또는 contact-gated coverage를 별도 health metric으로 유지하는 것이 좋음.
핵심 결론

현재 band의 가장 강한 내부-validity 질문은 80→160 convergence, 가장 강한 physical-validity 질문은 damage proxy와 contact/friction parameter sensitivity임. 반대로 recorder rescope, F=2 pad-field exceedance, realized-axis 도입 자체는 현재 형태로 band를 무너뜨릴 사안으로 보이지 않음.

Q1은 더 명확함. 이 commit에서 trustworthy soft-rigid pad force는 “어딘가의 hidden lambda를 읽는 것”이 아니라, solver가 사용하는 동일 _eval_soft_ef_contact()를 post-step state에서 재평가하여 pad별로 합산하는 것임. Newton 자체가 coupling feedback에 이미 정확히 그 방법을 사용하고 있음. 이 collector로 block-absent / static equilibrium / whole-soft-body momentum-balance 세 검사를 통과하면 W2 force channel을 GEOMETRY_ONLY에서 다시 승격시킬 근거가 충분히 강함.

