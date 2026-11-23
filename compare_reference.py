"""Compare MATLAB PCHIP/IK samples with the original Python reference."""
import json
import sys
from pathlib import Path

import numpy as np

here=Path(__file__).resolve().parent
root=here.parent
sys.path[:0]=[str(root/'dualarm_mpc_stage4'),
              str(root/'dualarm_mpc_stage2'/'dualarm_mpc')]
from model import ArmModel  # noqa: E402
from lift_reference import LiftReference  # noqa: E402

q0=np.array([1.1519853,-2.1270778,2.0490423,-1.4927645,1.5707890,-.4188037])
q=np.r_[q0,q0]
arms=[ArmModel(root/'lift_failure_review'/'actual_robot.urdf',side,
    fingers=(.01,.01),backend='numpy') for side in ('left','right')]
rod=np.eye(4);rod[:3,3]=[0,.29966,1.2717]
path=LiftReference(arms,q,rod)
matlab=json.loads((here/'.validation'/'reference.json').read_text(encoding='utf-8'))
samples=np.array([path.evaluate(t)[0] for t in (0,2,4,6,8)])
print('max_q_sample_diff_rad',float(np.max(abs(samples-np.array(matlab['q_samples'])))))
print('max_displacement',path.report()['max_joint_displacement_rad'],matlab['max_displacement'])
print('max_speed',path.report()['max_nominal_joint_speed_rad_s'],matlab['max_speed'])
print('min_sigma',path.report()['min_scaled_jacobian_singular_value'],matlab['min_sigma'])
