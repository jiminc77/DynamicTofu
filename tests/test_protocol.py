import copy, random, tempfile, unittest
import numpy as np
from src import judgment, bands, budget, profiles, io_schemas


def sample(t, **kw):
 d={"t":t,"jp":[1.0]*10,"bilateral_contact":True,"grasp_established":True}; d.update(kw); return d

class JudgmentTests(unittest.TestCase):
 def test_damage_latch_peak(self):
  r=judgment.evaluate([sample(0,jp=[1]*10),sample(1,jp=[1.06,1.06]+[1]*8),sample(2,jp=[1]*10)],0,2)
  self.assertIn("damage",r["labels"]); self.assertEqual(r["peak_damage_fraction"],.2)
 def test_contact(self):
  self.assertNotIn("drop",judgment.evaluate([sample(0,bilateral_contact=False),sample(.19,bilateral_contact=False)],0,1)["labels"])
  self.assertIn("drop",judgment.evaluate([sample(0,bilateral_contact=False),sample(.21,bilateral_contact=False)],0,1)["labels"])
 def test_displacement(self):
  for x,want in [(.019,False),(.02,False),(.021,True)]: self.assertEqual("drop" in judgment.evaluate([sample(0,relative_displacement_m=x)],0,1)["labels"],want)
 def test_slip_boundaries_and_or(self):
  for n,p,w in [(.0049,.0079,0),(.005,.008,0),(.0051,0,1),(0,.0081,1)]: self.assertEqual("slip" in judgment.evaluate([sample(0,slip_net_m=n,slip_peak_m=p)],0,1)["labels"],bool(w))
 def test_drop_suppresses_slip(self):
  self.assertEqual(judgment.evaluate([sample(0,relative_displacement_m=.03,slip_peak_m=.02)],0,1)["labels"],["drop"])
 def test_quiet_window_and_equalities(self):
  r=judgment.evaluate([sample(0,jp=[1.05]+[1]*9,relative_displacement_m=.03),sample(1,jp=[1.05]+[1]*9)],1,2)
  self.assertEqual(r["labels"],[])
 def test_timestamps_and_determinism(self):
  for ss in ([sample(1),sample(0)],[sample(0),sample(0)]):
   with self.assertRaises(ValueError): judgment.evaluate(ss,0,2)
  a=sample(0,jp=[1.1,1,1.1],particle_indices=[2,0,1]); b=sample(0,jp=[1,1.1,1.1],particle_indices=[0,1,2])
  self.assertEqual(judgment.evaluate([a],0,1),judgment.evaluate([b],0,1))

class BandTests(unittest.TestCase):
 def cell(self,k,n=3,invalid=0): return bands.reduce_cell([set()]*k+[{"x"}]*(n-k-invalid)+[None]*invalid,n)
 def test_seed_fixtures(self):
  for k in range(4): self.assertEqual(self.cell(k)["cell_pass"],k>=2)
  self.assertFalse(self.cell(3,5)["cell_pass"]); self.assertIsNone(self.cell(3,5,1)["cell_pass"]); self.assertIsNone(self.cell(3,5,2)["cell_pass"])
 def test_band_shapes(self):
  c={1:self.cell(2),2:self.cell(1),3:self.cell(2)}; r=bands.estimate_band(c); self.assertEqual(r["band_status"],"intermittent"); self.assertEqual(r["interior_failures"],[2])
  c[2]=self.cell(2,3,1); self.assertEqual(bands.estimate_band(c)["band_status"],"gapped")
  self.assertEqual(bands.estimate_band({1:self.cell(1)})["band_status"],"empty")
  r=bands.estimate_band({1:self.cell(2),2:self.cell(1)}); self.assertEqual(r["band_status"],"single_point"); self.assertTrue(r["censored_low"])
 def test_astar(self):
  e={"band_status":"empty"}; p={"band_status":"single_point"}
  self.assertEqual(bands.estimate_a_star({1:p,2:e,3:e})[0],2)
  self.assertEqual(bands.estimate_a_star({1:p,2:e,3:p})[1],"nonmonotone")
  self.assertEqual(bands.estimate_a_star({1:p,2:p})[1],"no_vanishing_observed")
  self.assertEqual(bands.estimate_a_star({1:p,2:e})[1],"censored_above")
 def mk(self,lo,hi,status="contiguous",cl=False,ch=False): return {"F_min":lo,"F_max":hi,"band_status":status,"censored_low":cl,"censored_high":ch,"cells":{}}
 def test_router_branches(self):
  empty={a:self.mk(None,None,"empty") for a in range(5)}; self.assertEqual(bands.shape_checkpoint(empty,[1,2])["outcome"],"scale_failure")
  van={a:self.mk(1,2) for a in range(5)}; van[4]=self.mk(None,None,"empty"); self.assertEqual(bands.shape_checkpoint(van,[1,2],4)["branch"],"band_vanishing")
  sparse={0:self.mk(1,2),1:self.mk(1,2,cl=True,ch=True),2:self.mk(1,2,cl=True,ch=True)}; self.assertEqual(bands.shape_checkpoint(sparse,[1,2])["branch"],"insufficient_usable_bands")
  struct={a:self.mk(a+1,5-a) for a in range(5)}; self.assertEqual(bands.shape_checkpoint(struct,[1,2,3,4,5,6])["outcome"],"structure_present")
  flat={a:self.mk(2,4) for a in range(5)}; self.assertEqual(bands.shape_checkpoint(flat,[1,2,3,4,5])["outcome"],"no_structure")
  flat[0]["band_status"]=flat[1]["band_status"]="intermittent"; r=bands.shape_checkpoint(flat,[1,2,3,4,5]); self.assertTrue(r["low_confidence"]); self.assertEqual(r["outcome"],"ambiguous")

class BudgetTests(unittest.TestCase):
 def test_named(self):
  r=budget.select_schedule(900,100); self.assertEqual(r["E1_budget"],86400); self.assertEqual(r["T_A_max"],921.6); self.assertFalse(r["escalate"])
  self.assertTrue(budget.select_schedule(930,100)["escalate"])
  t2=8*3600/(9*1.25); r=budget.select_schedule(800,t2); self.assertEqual(r["E1_budget"],72000); self.assertEqual(r["T_A_max"],768); self.assertTrue(r["escalate"])
 def test_random_equivalence_monotonic(self):
  random.seed(2)
  for _ in range(1000):
   T=random.random()*2000; e=random.random()*4000; r=budget.select_schedule(T,e)
   self.assertEqual(r["escalate"],T>r["T_A_max"]); self.assertEqual(r["reserved_e2_trials"],9)
  for T in (100,500,900):
   rs=[budget.select_schedule(T,e) for e in (0,1000,3000)]; self.assertGreaterEqual(rs[0]["T_A_max"],rs[1]["T_A_max"]); self.assertGreaterEqual(rs[1]["T_A_max"],rs[2]["T_A_max"])

class ProfileTests(unittest.TestCase):
 def test_all_contracts(self):
  for pid in profiles.PROFILE_IDS:
   r=profiles.generate(pid,3.0); self.assertEqual(r["profile_id"],pid); self.assertLessEqual(abs(r["realized_peak_accel"]-3)/3,.01)
   self.assertTrue(np.allclose(r["pos"][[0,-1]],r["endpoint_positions"],atol=1e-6))
   jerk=np.linalg.norm(np.diff(r["acc"],axis=0)/np.diff(r["t"])[:,None],axis=1)
   self.assertLessEqual(jerk.max(),r["j_max"]*1.01)
   if pid=="const_speed_arc": self.assertLessEqual(abs(3-r["speed"]**2/r["radius"]),3e-6)

class SchemaTests(unittest.TestCase):
 def config(self,e2=False):
  c={k:1 for k in io_schemas.COMMON_CONFIG_KEYS}; c.update(f_g_convention="per_finger_normal_mean",calibration={"slope":1,"intercept":0,"residual":0,"hysteresis":0})
  if e2: c.update({k:1 for k in io_schemas.E2_CONFIG_KEYS}); c.update(raw_field_recorded=True,raw_field_layout="concat+offsets",aggregates_derived_from_raw=True,taxel_binning="post_hoc_out_of_scope")
  return c
 def payload(self,s):
  if s=="e1.v1": return {k:1 for k in io_schemas.E1_PAYLOAD_KEYS}
  if s=="e1_band.v1": return {"rows":[],"a_star":None,"a_star_status":"no_vanishing_observed","coverage":{},"extra_replications":{}}
  return {}
 def test_roundtrip_and_every_missing_key(self):
  for s in ("e1.v1","e1_band.v1","e2.v1"):
   d=io_schemas.make(s,self.payload(s),self.config(s=="e2.v1"))
   with tempfile.NamedTemporaryFile(suffix=".json") as f: io_schemas.write_json(f.name,d); self.assertEqual(io_schemas.read_json(f.name),d)
   keys=io_schemas.COMMON_CONFIG_KEYS+(io_schemas.E2_CONFIG_KEYS if s=="e2.v1" else ())
   for key in keys:
    bad=copy.deepcopy(d); del bad["config"][key]
    with self.assertRaises(ValueError): io_schemas.validate(bad)

if __name__ == "__main__": unittest.main()
