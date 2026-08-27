# External consult 2: GPT-5 Pro on the VBD contact failure (2026-08-28)

- Chat: "VBD Grasp Slip Analysis" — https://chatgpt.com/c/6a906b24-9384-83ee-b061-151767e1de5f (user account, Pro mode, worked 16m29s, read the repo at 3f1eb88 + Newton source at pin b74df534)
- Verbatim transcript below (Korean/English as produced).

결론부터 말하면, 현재 stiff-block ejection의 1순위 원인은 contact stiffness 자체보다 SolverCoupledProxy에서의 actuator semantics mismatch로 판단됨.

현재 rig는 MuJoCo 쪽 finger joint를 effort_limit=0.45 N로 제한하지만, 같은 joint를 VBD destination의 proxy joint로도 활성화하고 target을 복사함. 문제는 pinned SolverVBD가 명시적으로 joint_effort_limit를 지원하지 않는다는 점임. 즉 MuJoCo source는 0.45 N actuator인데 VBD proxy finger는 사실상 uncapped target_ke=2e4 N/m, target_kd=200 drive임. 현재 geometry에서 contact onset 대비 final target error가 약 10–17 mm라서 VBD proxy가 순간적으로 200–340 N/finger급 drive demand를 만들 수 있음. 의도한 0.45 N보다 400–750× 큼. 이 상태에서는 kf, VBD iterations, 심지어 contact ke를 조금 바꾸는 것으로 oracle이 정상화되기 어려움.

또 하나의 독립 문제는 VBD rigid-soft friction임. 이 build에서 soft_contact_kf는 이 path에 사용되지 않음. friction은 persistent static-friction spring이 아니라 friction_epsilon으로 regularize된 velocity/displacement-level Coulomb law라서, Coulomb cone 안에서도 finite creep velocity가 생김. 따라서 ejection을 고친 뒤에도 현재 default friction_epsilon=1e-2 m/s라면 5 s hold slip은 예상되는 동작임.

그리고 repo 기준 한 가지 정정 필요. committed src/vbd_rig.py는 BLOCK_DIM=4, BLOCK_CELL=10 mm라서 5 vertices/edge임. 8 mm면 5 cells → 6 vertices/edge가 됨. local branch에서 이미 8 mm로 바꾼 것이 아니라면 현재 실제 resolution은 10 mm임. 또한 사용자가 지목한 scripts/probes/diag_rig.py는 여전히 이전 MPM diagnostic이고, 현재 VBD rig의 핵심 구현은 src/vbd_rig.py에 있음.

(a) stiff-block ejection root cause ranking
1. Proxy joint에서 effort_limit가 사라지는 문제 — 가장 유력

현재 구조:

MuJoCo:
  target_ke = 2e4
  target_kd = 200
  effort_limit = 0.45 N

       ↓ proxy body + proxy joint

VBD:
  target_ke = 2e4
  target_kd = 200
  effort_limit = NOT SUPPORTED

SolverCoupledProxy.Proxy(... joints=self.gripper_joints ...)는 destination에서 해당 joint를 유지하고 1-DoF drive targets를 복사함. 반면 SolverVBD 문서에는 joint_effort_limit가 unsupported라고 명시돼 있음. base example_proxy_joint_gripper.py가 이 문제를 못 드러낸 이유도 일관됨. 그 example은 zero gravity이고 finger effort_limit가 250–500 N이며, sub-Newton force-controlled oracle을 검증하지 않음.

현재 geometry를 보면 더 명확함.

finger body center open position: y=75 mm

pad half-thickness: 6 mm

tofu face: 20 mm

particle radius: 7 mm

particle-sphere contact onset은 대략

q
touch
	​

≈75−(20+6+7)=42 mm.

그런데 code의 final target은

q
target
	​

=75−20+4=59 mm.

즉 contact 이후에도 약 17 mm를 더 닫으라는 drive임. VBD proxy joint에서 단순 stiffness demand만 계산해도

F
drive
	​

∼2×10
4
⋅0.017≈340 N.

particle radius를 제외한 mesh/pad geometry만 보더라도 over-target이 약 10 mm → 200 N 수준임.

가장 좋은 discriminator: 기존 oracle에서 딱 한 가지를 바꿔 Proxy(... joints=())로 body proxies만 전달. 나머지 E=1 MPa, 0.45 N, trajectory, contact params 동일. ejection이 즉시 사라지거나 수십 배 감소하면 거의 확정임.

동시에 log할 값은 source MuJoCo finger coordinate/force와 VBD proxy finger displacement/contact force. intended 0.45 N closure인데 destination contact normal이 수 N~수백 N spike를 보이면 확정.

2. Lagged proxy feedback + stiff contact — 매우 유력한 증폭 메커니즘

mode="lagged"는 source의 begin pose + end velocity를 destination proxy로 보내고, destination contact response를 harvest한 뒤 source에 다음 pass/step의 force로 돌려줌. Newton은 rewind logic으로 명시적 double-counting을 피하려고 하지만, contact reaction 자체는 여전히 time-lagged feedback임. 따라서 "contact를 두 번 적분한다"기보다는 delayed stiff feedback loop라고 보는 것이 정확함.

현재 actual per-contact normal ke도 생각보다 큼. 이유는 material mixing임.

soft_contact_ke = 50 kN/m
pad shape ke    = 80 kN/m

VBD body-particle contact는 arithmetic mean이므로

k
e
contact
	​

=65000 N/m/contact.

20개의 feature가 동시에 contact하면 aggregate penalty stiffness가 대략 1.3 MN/m.

finger pad 자체 mass를 geometry/density로 계산하면 약 15 g. substeps=12에서

dt=1/(60⋅12)=1.389 ms.

lagged-coupling stability scale인

m/dt
2

는 약 7.8 kN/m에 불과함. 1.3 MN/m와 비교하면 두 order 이상 차이. 이것이 substeps 24에서 ejection 대신 velocity spike로 바뀐 관측과 잘 맞음.

Discriminator: actuator mismatch를 먼저 제거한 뒤 lagged vs staggered, proxy_relaxation={1,0.5,0.25}, mass_scale={1,4} 비교. launch amplitude가 relaxation/mass에 강하게 반응하면 proxy loop 확인.

3. particle radius + excessive target penetration

particle_radius는 "mesh resolution" knob가 아니라 contact equation에 직접 들어가는 geometric contact thickness임.

Pinned VBD는

δ=−[n⋅(x
p
	​

−x
c
	​

)−r
p
	​

−margin]

로 penetration을 계산함. 즉 r=7–12 mm는 40 mm cube에서 상당히 큰 inflation임.

특히 stabilization probe에서 radius를 12 mm로 키우는 것은 tunneling을 막는 대신 이 문제에서는 contact onset을 훨씬 앞당기므로, 현재 59 mm closure target과 결합하면 오히려 proxy joint target error가 커짐.

Discriminator: r=0.4h와 corrected q_touch를 동시에 적용하고 first-contact max penetration 기록. r을 줄였을 때 contact onset과 force spike timing이 정확히 이동하면 확인.

4. per-contact stiff penalty × dt / mass ratio

중요하지만 위 두 문제가 해결된 후 판단해야 함.

Newton body-particle VBD contact는 normal force가 실제로

F
n
	​

=k
e
	​

δ

인 penalty force임. damping도 closing direction에서 kd v_n 형태. 따라서 ke=65 kN/m/contact는 0.45 N pinch에 지나치게 높은 값임.

Discriminator: 모든 material mixing을 제거하고 actual mixed ke={1,3,10} kN/m로 동일 force oracle 실행. ejection velocity가 ke와 단조적으로 변해야 penalty-driven instability.

5. sparse vertex contact / discontinuous contact set

stiff ejection의 1차 원인보다는 tofu slip에서 더 중요함. 다만 coarse face에서 새로운 vertex가 갑자기 contact set에 들어오면 normal force가 discontinuously 생겨 spike를 만들 수 있음.

Pinned Newton에는 이 문제를 위한 enable_rigid_soft_full_surface_contact=True가 있고, 더 중요한 점은 정확히 parallel-jaw VBD grasp/lift/hold example인 example_vbd_gripper_soft_grid.py가 이 flag를 사용한다는 점임. Example 설명 자체가 per-particle mode에서는 object가 slip/fall하고, full-surface edge contact를 켜야 grip된다고 명시함.

6. ALM/compliant-contact setting

낮은 우선순위.

rigid_compliant_alm=True는 rigid body-body contact와 structural/joint constraints 쪽의 formulation임. body-particle path는 별도의 body_particle_contact_penalty_k를 사용하고 per-iteration penalty state를 갱신함. 따라서 이 flag를 바꿔서 soft-rigid contact 자체가 persistent ALM/static-friction contact로 변하는 것이 아님.

다만 proxy joint drive에는 rigid_compliant_alm가 관여하므로 joint-induced launch amplitude는 변할 수 있음. 따라서 ALM 변화로 ejection이 달라져도 "contact ALM 문제"라고 해석하면 안 됨.

7. pinned commit 자체의 bug

현재 evidence로는 마지막 순위.

같은 b74df534 안에 direct single-SolverVBD rigid-soft integration과 direct VBD gripper lift/hold examples가 존재함. example_rigid_soft_contact.py도 rigid와 soft를 하나의 VBD solver에 넣는 mode를 지원함. 따라서 material/contact engine 자체가 이 regime을 원천적으로 못 처리한다고 볼 근거는 없음.

Pin bug 판정 조건: corrected geometry + no proxy joint drive + full-surface contact의 minimal 1-DOF oracle에서도 동일 ejection이 재현되고, 같은 scene이 newer Newton에서 사라지는 경우.

(b) soft_contact_ke, kd, kf, mu 정확한 semantics
soft_contact_ke

단위 N/m per contact record. pressure stiffness도 아니고 pad-area-normalized stiffness도 아님.

Rigid shape의 shape_material_ke와 arithmetic average:

k
e
pair
	​

=
2
k
e
soft
	​

+k
e
shape
	​

	​

.

현재:

(50k+80k)/2=65k N/m/contact.

Normal force:

F
n
	​

=k
e
pair
	​

δ.

따라서 mesh refinement 시 같은 soft_contact_ke를 유지하면 contact count 증가에 따라 aggregate patch stiffness도 증가함. 이것이 중요한 scaling issue임.

soft_contact_kd

단위 N·s/m per contact. rigid shape kd와 arithmetic mean.

현재:

k
d
pair
	​

=(10
−3
+10
−4
)/2=5.5×10
−4
 Ns/m.

사실상 zero damping임.

Closing contact에서:

F
d
	​

≃−k
d
	​

v
n
	​

.

Normal contact의 critical-damping scale은 대략

c
c
	​

=2
km
eff
	​

	​

.

5 mm mesh의 surface-node effective mass를 ~10^-4 kg, k=1–8 kN/m로 보면 c_c≈0.6–1.8 N·s/m. 따라서 current 5.5e-4는 약 3 orders 작음.

첫 probe는 양쪽 material 모두 kd=0.5–1.0 N·s/m; stiff oracle에서 필요하면 2.0까지 권장.

soft_contact_kf

SolverVBD rigid-soft path에서는 사용되지 않음.

이것이 {1e3,1e4,1e5} ladder가 완전히 invariant했던 이유임. kf를 더 sweep할 이유 없음. SolverSemiImplicit 등의 다른 contact path에서 의미가 있더라도 현재 VBD body-particle law의 tangential stiffness knob가 아님.

soft_contact_mu

Rigid shape mu와 geometric mean:

μ
pair
	​

=
μ
soft
	​

μ
shape
	​

	​

.

현재 pad mu=0.7이므로:

configured soft μ	actual pair μ
0.5	0.592
1.0	0.837
2.0	1.183

따라서 2 N/finger, soft mu=2 case의 actual nominal bilateral Coulomb capacity는

2(2)(1.183)=4.73 N,

weight 0.628 N 대비 7.54×, 12×가 아님. 그래도 충분히 큰 margin이므로 핵심 conclusion은 유지됨.

Oracle도 마찬가지:

2(0.45)(0.837)=0.753 N,

weight 대비 1.20× margin밖에 없음.

실험 semantics를 깨끗하게 하려면 desired mu=1이면 soft와 pad 둘 다 1.0으로 설정할 것. ke, kd도 같은 이유로 양쪽을 동일하게 맞추는 것이 좋음.

friction은 static spring이 아님

VBD body-particle friction은 current-step tangential relative displacement

u
t
	​

=v
t
	​

dt

를 이용한 regularized Coulomb force임. Default

Python
Run
friction_epsilon = 1e-2  # m/s

이고 smoothing displacement는

ϵ
u
	​

=ϵ
v
	​

dt.

x=v_t/epsilon_v ≤1이면 friction magnitude가

∣F
t
	​

∣=μF
n
	​

(2x−x
2
),

v_t≥epsilon_v에서 full Coulomb μFn.

즉 zero-slip에서 finite tangential load를 버티는 persistent stick state가 없음. 하중을 받으려면 일정 creep velocity가 필요함.

64 g cube에서 한 finger가 부담해야 하는 gravity는

F
t
	​

=mg/2=0.314N.

Oracle Fn=.45, μ=1이면 필요한 Coulomb fraction

r=0.314/0.45=0.698.

Regularized branch의 steady velocity는

x=1−
1−r
	​

=0.450.

따라서 default epsilon이면

v
creep
	​

=0.450(0.01)=4.5 mm/s.

5 s면 22.5 mm임.

현재 mixed μ=0.837이면 약 5.9 mm/s, 즉 5 s에 거의 30 mm.

따라서 ejection을 제거해도 current friction setting으로 oracle hold가 실패하는 것은 충분히 예상됨.

5 s 동안 slip을 1 mm 이하로 만들려면:

ϵ
v
	​

<
0.450
0.2mm/s
	​

≈4.4×10
−4
 m/s.

권장 starting point:

Python
Run
friction_epsilon = 2e-4   # m/s

안정성이 민감하면 5e-4, 최종 hold에는 1–2e-4.

단, epsilon을 낮추면 tangential Hessian이 강해지므로 lagged proxy보다 unified VBD에서 훨씬 안전함.

normal ke scaling

사용자 pad active area A≈3 cm²=3e-4 m², thickness scale L≈40 mm라 놓으면 material patch stiffness order는

K
mat
	​

∼
L
EA
	​

.
E	EA/L
25 kPa	188 N/m
100 kPa	750 N/m
200 kPa	1.5 kN/m
500 kPa	3.75 kN/m
1 MPa	7.5 kN/m

Contact penalty aggregate stiffness가 material보다 대략 5–10× 높으면 충분함. active contact count Nc≈10–30이면

k
e,contact
	​

∼
N
c
	​

5–10K
mat
	​

	​

.

실제 빠른-start 값으로는, soft와 pad ke를 둘 다 동일하게:

E	recommended pair ke
7–25 kPa	0.5–1.5e3 N/m
100 kPa	1–2e3
200 kPa	2–3e3
500 kPa	3–5e3
1 MPa	5–8e3

현재 65e3/contact보다 1–2 orders 낮음.

중요한 점: soft_contact_ke만 낮추고 pad ke=8e4를 그대로 두면 arithmetic averaging 때문에 lower bound가 거의 4e4가 됨. pad ShapeConfig.ke도 같이 내려야 함.

(c) Coulomb oracle을 통과시키는 구체 recipe
먼저 E sweep이 아니라 architecture/control bug를 고칠 것

100/200/500 kPa sweep 자체는 좋은 probe지만, 현재 stack 그대로 하면 "material stability envelope"가 아니라 uncapped proxy drive envelope를 측정하게 됨.

추천 oracle configuration
Architecture:
  single SolverVBD       <-- preferred

Object:
  E = 100 kPa first
  nu = 0.30
  rho = 1000
  then 200, 500 kPa, 1 MPa

Mesh:
  h = 8 or 10 mm for first diagnostic
  production h = 5 mm

particle_radius:
  0.4 h
  e.g. h=8 mm -> 3.2 mm

Contact:
  soft ke = pad ke = 2 kN/m @100 kPa
                        3 kN/m @200 kPa
                       5 kN/m @500 kPa
                       8 kN/m @1 MPa
  soft kd = pad kd = 1 N s/m
  soft mu = pad mu = 1.0
  friction_epsilon = 2e-4 m/s

Collision:
  enable_rigid_soft_full_surface_contact=True
  soft_contact_margin ~1 mm

Solver:
  rigid_compliant_alm=True
  substeps=20
  VBD iterations=10 initially
  then 20 for convergence check

Pinned Newton의 direct VBD gripper example이 20 substeps, 5 iterations, gravity, unified rigid+soft, full-surface contact로 실제 lift/hold를 수행함. 따라서 이 architecture는 pin에서 지원되는 path임.

Force control

Pure VBD에서는 joint_effort_limit에 의존하면 안 됨. SolverVBD는 대신 Control.joint_f를 지원함.

Finger prismatic drive의 target_ke=0, target_kd=0로 두고:

Python
Run
control.joint_f[left_dof]  = 0.45
control.joint_f[right_dof] = 0.45

현재 joint coordinates는 양쪽 모두 positive q가 inward closure이므로 같은 positive sign이 맞을 가능성이 높지만, 첫 frame에서 확인할 것.

Finger limits는 geometric safety stop으로만 사용.

Lift Z는 target PD를 그대로 써도 됨.

closure trajectory

force actuation을 0→0.45 N으로 0.5–1.0 s ramp.

이후:

1 s preload/settle.

measured contact normal force가 0.45±0.05 N/finger.

50 mm lift를 2 s에 수행.

suspended 5 s hold.

Acceptance criteria

Oracle PASS를 단순 "안 날아감"으로 잡으면 안 됨.

first contact부터 launch 없음: cube COM excursion <5 mm before lift.

peak COM speed during settle <0.2 m/s; hold에서는 <5 mm/s, ideally <1 mm/s.

averaged normal force: 0.45±10% N/finger.

post-lift ground contact zero.

relative tofu/gripper vertical drift over final 5 s: <1 mm preferred, <2 mm hard maximum.

contact maintained on both pads >95% of hold frames.

doubled substeps (20→40 또는 최소 20→24)에서 final drift 변화 <20%, mean Fn 변화 <10%.

finite vertices and positive tet volumes.

그리고 full-surface contact를 반드시 비교할 것. Newton 자체의 VBD gripper example이 이것을 slip vs hold를 가르는 기능으로 사용함.

E=1 MPa가 VBD 한계인가?

아님.

example_softbody_franka.py는 pinned code에서 k_mu≈1e6, k_lambda≈1e6 scale soft body를 VBD로 돌림. Contact architecture는 kinematic/external rigid라 oracle과 동일하지 않지만, 최소한 1 MPa급 material stiffness 자체가 VBD의 근본적인 forbidden regime이 아님을 보여줌.

예상은 다음과 같음.

현재 MuJoCo+lagged proxy + proxy joints: 100–500 kPa에서도 sensitivity 가능.

unified VBD, h≈5 mm, substeps 20–24: 1 MPa, nu=.3는 충분히 plausible.

1 MPa만 실패하고 500 kPa까지 clean하면 그때 iteration/dt conditioning을 조사할 가치가 있음.

따라서 moderate oracle 순서는 맞지만:

100 kPa → 200 kPa → 500 kPa → 1 MPa

로 하고, 100 kPa도 못 통과하면 E를 더 내리지 말고 contact/control architecture 문제로 판정해야 함.

(d) tofu가 2 N, μ=2에서도 slip하는 이유

여기서는 여러 효과가 겹쳐 있음.

1. kf가 낮아서가 아님

현재 VBD path에서 kf는 효과가 없음. 이것은 제거 가능.

2. default friction regularization만으로 수 mm creep가 예정됨

Fn=2 N, actual μ=1.183이면 한 pad의 required load fraction:

r=0.314/(2⋅1.183)=0.133.

따라서

v
creep
	​

≈0.0687ϵ=0.687 mm/s

for epsilon 1e-2.

5 s 동안 3.4 mm.

3–4 mm만 tangentially 이동해도 coarse contact patch에서는 active vertices가 바뀌고 bulged face의 local normal도 변함. 이후 geometric escape가 시작되면 nominal μFn argument 자체가 무너짐.

3. 5 vertices/edge는 pad-scale static contact에 너무 sparse

현재 repo의 10 mm spacing이면 사용자가 말한 active ~3 cm² patch의 characteristic width가 약 17 mm이므로 방향당 고작 2개 정도 sample spacing.

contact가 중앙 정렬되느냐에 따라 실제 active vertices가 4개 안팎으로 떨어질 수 있음.

Newton의 full-surface contact 기능이 이 문제를 직접 겨냥함. per-vertex뿐 아니라 edge/face records를 추가해야 함.

4. 현재 radius와 closure target이 지나치게 aggressive

r=7 mm, h=10 mm에 target이 contact onset보다 17 mm 안쪽임.

Soft tofu에서는 stiff block처럼 즉시 eject하지 않고, proxy drive energy를 bulging/extrusion으로 흡수함. 8 N에서 bbox +60%는 이 해석과 정확히 맞음. Repo v2 결과에서도 low force pure slip, high force bulge/extrusion bifurcation이 이미 나타남.

5. 5–8 N은 tofu에서 물리적으로도 너무 큼

3 cm² 실제 patch라면:

8 N:

σ=8/(3×10
−4
)=26.7kPa.

E=25 kPa라면 단순 nominal strain만도

σ/E≈107%.

2 N도 약:

6.67kPa/25kPa=27%

nominal strain.

따라서 8 N extrusion은 numerical pathology만으로 볼 수 없음. 25 kPa tofu에서 intact grasp의 meaningful force band는 훨씬 아래일 가능성이 큼.

Static friction을 정상화하면 필요한 force 자체는 작음.

μ=1에서 minimum:

F
n
	​

>
2μ
mg
	​

=0.314N/finger.

1.5× safety factor라 해도 ~0.47 N/finger.

따라서 production sweep은 대략:

0.4, 0.6, 0.8, 1.0, 1.2 N/finger

부터 시작하고, 2 N 이상은 damage/extrusion branch용으로 두는 편이 맞음.

static friction을 실제로 복구하는 조합

soft μ = pad μ = desired μ.

friction_epsilon = 2e-4 … 5e-4 m/s.

enable_rigid_soft_full_surface_contact=True.

h=5 mm.

radius=2–2.5 mm.

contact ke tofu ~0.5–1.5 kN/m/contact.

kd ~0.5–1 N·s/m.

force-controlled closure, no deep position target.

2–3 s lift, not abrupt lift.

hold metric은 world-frame COM이 아니라 COM relative to gripper로 평가.

이 조합 후에도 constant normal force에서 shear가 지속되면 그때부터 material creep/extrusion 문제라고 판단하는 것이 맞음.

(e) architecture decision

순위는 명확함.

1위: option (2), unified pure SolverVBD

deadline 기준 가장 높은 성공 확률.

Pinned Newton에 이미 정확히 가까운 precedent가 있음:

rigid gripper + soft object

gravity

single SolverVBD

parallel jaw close

lift

suspended hold

full-surface rigid-soft contact

인 example_vbd_gripper_soft_grid.py.

또 example_rigid_soft_contact.py도 unified VBD rigid+soft mode를 제공함.

장점:

lagged rigid↔soft feedback 제거.

MuJoCo↔VBD actuator mismatch 제거.

contact + joint + elasticity가 한 nonlinear solve에 존재.

principal strain을 tet deformation gradient에서 직접 계산하기 쉬움.

Control.joint_f로 true sub-Newton force closure 구현 가능.

4-day deadline에서는 이것이 최우선.

2위: option (1), MuJoCo + VBD proxy 유지

다음 조건을 전부 적용하는 경우만.

proxy joint drive 제거 또는 semantics 수정.

body-only proxy test.

deep close target 제거.

staggered 비교.

proxy_relaxation≈0.25–0.5.

contact ke 1–2 orders 감소.

full-surface contact.

epsilon 감소.

현재 상태에서 contact ke만 계속 sweep하는 것은 권하지 않음.

특히 proxy iterations를 8로 올리는 전략은 피할 것. MPM에서도 iteration-dependent blowup이 이미 있었고, VBD proxy source도 같은 delayed-feedback architecture임.

3위: MPM 복귀

Gate A의 elastic Coulomb oracle pass는 중요한 장점임. MPM contact stack 자체는 검증됨. 반면 real-tofu regime은 current implementation에서 바로 필요한 rheology axis가 아직 unstable임: η=2e5 Pa·s effort runs가 blow-up했고, proxy iterations 8도 blow-up.

Newton source를 보면 SolverImplicitMPM은 viscosity를 implicit rheology solve에 넣음. material η는 내부적으로 η/dt로 yield parameters에 들어감. Solver 문서도 stiff/inelastic regime에서 timestep에 대해 implicit stability를 목표로 한다고 설명함. 따라서 η=2e5 failure를 "viscous explicit CFL violation"로 해석하면 안 됨. 현재 더 가능성 높은 것은 rheology convergence + rigid proxy coupling instability임.

MPM으로 fallback해야 한다면:

Python
Run
cfg.max_iterations = 250  # 필요하면 500
cfg.tolerance = 1e-6
cfg.velocity_basis = "Q1"
cfg.strain_basis = "P0"
cfg.collider_basis = "S2"
cfg.warmstart_mode = "particles"  # or auto if equivalent

Newton의 viscous reference example도 max_iterations=250, tolerance=1e-6, Q1/P0/S2를 사용함.

그리고:

proxy iterations 1.

current coupled dt를 최소 2–4× 줄여 feedback loop 안정화.

η={2e5,5e5,1e6} bracket.

rheology residual/convergence 반드시 기록.

필요하면 solver sequence ("conjugate-residual", "gauss-seidel")를 시험. Newton Config는 solver chaining/warmstart를 지원함.

필요한 viscosity order

단순 timescale estimate:

τ∼η/E.

η=20 Pa·s라면:

E=7 kPa: τ≈2.9 ms

E=25 kPa: τ≈0.8 ms

5 s hold에서는 사실상 zero-viscosity와 다름없음.

η=2e5:

E=7 kPa: τ≈29 s

E=25 kPa: τ≈8 s.

따라서 5 s hold를 다루기 위한 plausible range는 적어도 **2e5–1e6 Pa·s**임. 20 Pa·s는 4–5 orders 작다는 기존 판단이 맞음.

단, Newton의 mpm.viscosity는 단순 Maxwell dashpot가 아니라 plastic flow-rule viscosity이므로 이 η/E는 calibration의 order-of-magnitude guide일 뿐 exact constitutive mapping은 아님.

2 GPU-days 안에는 MPM high-viscosity stabilization 연구보다 VBD contact/control 수정이 훨씬 bounded problem임.

(f) mesh/contact resolution

현재 4 cm cube 기준:

cell h	cells/edge	vertices/edge	total vertices
10 mm	4	5	125
8 mm	5	6	216
5 mm	8	9	729
4 mm	10	11	1331

3 cm² patch characteristic width는 약 17 mm.

h=10 mm: 약 1.7 intervals across patch → 불충분.

h=8 mm: 약 2.2 → diagnostic 정도.

h=5 mm: 약 3.5 → minimum credible production.

h=4 mm: 약 4.3 → contact/strain convergence check에 적절.

추천: production sweep h=5 mm, representative 2–3 forces만 h=4 mm convergence.

Particle radius는 full-surface contact를 사용할 경우:

r≈0.35–0.5h.

따라서:

h	radius
8 mm	3–4 mm
5 mm	1.8–2.5 mm
4 mm	1.5–2 mm

현재 6–8 mm는 지나치게 큼. sparse per-particle contact를 radius로 메우려 하지 말고 full-surface edge/face contact로 sampling hole을 메우는 것이 맞음. Newton의 direct VBD gripper example도 바로 이 방식을 사용함.

Computational cost는 volume DOF 기준 대략 h^-3, surface contacts는 h^-2.

10 mm를 1×로 보면:

h	volumetric work approx.	surface-contact count approx.
10 mm	1×	1×
8 mm	1.95×	1.56×
5 mm	8×	4×
4 mm	15.6×	6.25×

따라서 모든 development run을 4 mm로 돌리는 것은 GPU budget 낭비. mechanism triage 8–10 mm → sweep 5 mm → convergence points 4 mm가 적절함.

Principal-strain damage metric

VBD가 이 목표에는 MPM보다 깔끔함.

각 tet에서 rest matrix Dm, current Ds:

F=D
s
	​

D
m
−1
	​

.

Damage metric은 예를 들어 Green-Lagrange strain

E
G
	​

=
2
1
	​

(F
T
F−I)

의 최대 eigenvalue:

ϵ
1
	​

=λ
max
	​

(E
G
	​

).

Grip-force sweep마다 최소 다음 세 값을 저장 권장:

max ε1

volume-weighted P99 ε1

volume fraction(ε1 > ε_damage)

contact node 하나의 mesh artifact 때문에 max 하나만 damage 판정을 지배하게 하면 안 됨. 최종 threshold result는 h=5 vs 4 mm에서 P99 또는 damaged volume fraction이 수렴하는지 확인하는 것이 더 defensible함.

Prioritized 2-day triage
Day 1 — oracle을 정상화

P0. 현재 proxy hypothesis를 1–2시간 내에 falsify.

현재 stiff oracle 그대로 두고 세 run만 실행:

current proxy joints.

Proxy(... joints=()).

pure unified SolverVBD.

각 run에서 source finger q, proxy finger q, per-pad summed Fn, maximum contact penetration, COM velocity를 기록. 2번에서 launch가 사라지면 proxy effort-limit mismatch 확정.

동시에 q_target=59 mm를 폐기. diagnostic position closure가 필요하면 current geometry에서 q≈42–43 mm 이상 깊게 밀지 말 것. Force-control run에서는 deep target 자체를 없앨 것.

P1. Pure-VBD Coulomb oracle 구축.

먼저 E=100 kPa:

h = 8 mm
r = 3 mm
ke soft/pad = 2e3
kd soft/pad = 1
mu soft/pad = 1
friction_epsilon = 2e-4
full_surface_contact = True
substeps = 20
iterations = 10
Fn = 0.45 N/finger

PASS하면 200 → 500 → 1000 kPa.

1 MPa만 실패하면 iterations=20, substeps=24, ke=5–8k bracket.

Hard gate: 100–200 kPa oracle도 5 s 동안 <2 mm relative slip을 못 하면 tofu sweep으로 넘어가지 말 것.

Day 2 — tofu band + damage sweep

Oracle이 pass한 정확한 contact/control architecture를 고정.

Production:

h = 5 mm
r = 2–2.5 mm
E = 7, 15, 25 kPa
nu = 0.45
ke = 0.5–1.5 kN/m
kd = 0.5–1
μ_pair target = 1.0  # both materials = 1
epsilon = 2e-4
substeps = 20–24
lift = 50 mm over 2–3 s
hold = 5 s

Grip-force sweep:

0.4, 0.6, 0.8, 1.0, 1.2 N/finger

필요할 때만 1.5, 2.0 N.

각 arm에서:

hold success
relative slip over 5 s
mean / peak Fn
contact count
bbox deformation
max principal strain
P99 principal strain
damaged-volume fraction

기록.

마지막으로 0.6 N 근처의 intact case와 damage-onset 근처 case 두 개만 h=4 mm로 재실행해 mesh convergence 확인.

Architecture go/no-go 기준: Day 1 중반까지 pure VBD oracle이 100–500 kPa에서 통과하면 그대로 VBD 고정. Pure VBD는 통과하지 않지만 body-only MuJoCo proxy가 통과하면 option (1) 사용. 둘 다 실패할 때만 Gate-A-validated MPM으로 돌아가고, 그 경우 η=2e5–1e6, implicit rheology tol=1e-6, 250–500 iterations, proxy iterations 1, smaller coupled dt를 사용하는 것이 2 GPU-days에서 가장 합리적인 fallback임.

현재 evidence 기준으로는 MPM으로 복귀하기 전에 proxy joint effort-limit mismatch + friction_epsilon + full-surface contact 세 항목을 먼저 고치는 것이 압도적으로 높은-value path임.