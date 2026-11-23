"""Cross-check the MATLAB base implementation against the original NumPy model."""
import json
import sys
from pathlib import Path

import numpy as np

here = Path(__file__).resolve().parent
root = here.parent
sys.path.insert(0, str(root/'dualarm_mpc_stage2'/'dualarm_mpc'))
from model import ArmModel  # noqa: E402

q=np.array([1.1519853,-2.1270778,2.0490423,-1.4927645,1.5707890,-0.4188037])
matlab=json.loads((here/'.validation'/'model.json').read_text(encoding='utf-8'))
for result in matlab:
    arm=ArmModel(root/'lift_failure_review'/'actual_robot.urdf',
        result['side'],fingers=(.01,.01),backend='numpy')
    mass,gravity,jac,tip=arm.evaluate(q,np.zeros(6))
    errors={
        'mass':float(np.max(np.abs(np.array(result['mass'])-mass))),
        'gravity':float(np.max(np.abs(np.array(result['gravity']).ravel()-gravity))),
        'jacobian':float(np.max(np.abs(np.array(result['jacobian'])-jac))),
        'tip':float(np.max(np.abs(np.array(result['tip'])-tip))),
    }
    print(result['side'],errors)
    assert max(errors.values())<1e-9
