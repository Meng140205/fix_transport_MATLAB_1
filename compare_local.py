"""Cross-check MATLAB constrained dynamics against the Python source model."""
import json
import sys
import types
from pathlib import Path

import numpy as np

here=Path(__file__).resolve().parent
root=here.parent
sys.path[:0]=[str(root/'dualarm_mpc_stage4_1'),
              str(root/'dualarm_mpc_stage2_2'),
              str(root/'dualarm_mpc_stage2'/'dualarm_mpc')]
sys.modules['osqp']=types.SimpleNamespace()
from model import ArmModel,rod_inertia  # noqa: E402
from tracking_qp import local_model,solve_mpc  # noqa: E402

q0=np.array([1.1519853,-2.1270778,2.0490423,-1.4927645,1.570789,-.4188037])
q=np.r_[q0,q0];dq=np.zeros(12)
arms=[ArmModel(root/'lift_failure_review'/'actual_robot.urdf',side,
    fingers=(.01,.01),backend='numpy') for side in ('left','right')]
rod=np.eye(4);rod[:3,3]=[0,.29966,1.2717]
grasp=[np.linalg.inv(rod)@arms[i].evaluate(q[i*6:i*6+6],np.zeros(6))[3]
       for i in range(2)]
payload=rod_inertia(root/'mpc_readonly_review_2026-11-16'/'rod_with_pose.sdf')
model=local_model(arms,q,dq,rod,np.zeros(6),grasp,payload,joint_friction=np.zeros(12))
matlab=json.loads((here/'.validation'/'local.json').read_text(encoding='utf-8'))
for key,original in (('F',model['F']),('g',model['g']),
                     ('A',model['Aclosure']),('error',model['error']),
                     ('L',model['L']),('l',model['l'])):
    error=float(np.max(abs(np.asarray(matlab[key]).reshape(np.shape(original))-original)))
    print(key,error)
    assert error<1e-8
previous=np.r_[arms[0].evaluate(q[:6],np.zeros(6))[1],
               arms[1].evaluate(q[6:],np.zeros(6))[1]]
lower=np.r_[arms[0].lower,arms[1].lower]
upper=np.r_[arms[0].upper,arms[1].upper]
limits=np.r_[arms[0].effort,arms[1].effort]
solution=solve_mpc(model,q,np.zeros((11,36)),previous,lower,upper,limits,
                   horizon=10,dt=.05)
print('QP methods',matlab['qp_method'],solution['method'])
print('QP torque max error',float(np.max(abs(np.array(matlab['qp_torque']).ravel()-solution['torque']))))
