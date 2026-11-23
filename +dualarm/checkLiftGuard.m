function metrics=checkLiftGuard(cache,path,arms,grasp,initialFingers,...
    motionStart,height,checkGrasp)
%CHECKLIFTGUARD Stage 4 trajectory, grasp, state and controller guards.
if nargin<8,checkGrasp=false;end
if ~cache.ready('rod'),error('State/status stale during lift');end
statuses={cache.statusLeft,cache.statusRight};
for i=1:2
    s=statuses{i};
    if ~s.state_valid || s.mode==3 || s.torque_limited
        error('Controller invalid, faulted or torque-limited');
    end
end
q=cache.q;dq=cache.dq;rod=cache.rod;
if any(~isfinite([q;dq;cache.fingers;rod(:);cache.rodVelocity]))
    error('Nonfinite trajectory feedback');
end
if isempty(motionStart),elapsed=-1;else,elapsed=cache.simTime-motionStart;end
[qref,~,target,~]=dualarm.evalLiftReference(path,elapsed);
jointError=max(abs(q-qref));
poseError=norm(rod(1:3,4)-target(1:3,4));
angle=norm(dualarm.rotLog(rod(1:3,1:3)*target(1:3,1:3).'));
if jointError>.02 || max(abs(dq))>.10
    error('Joint tracking guard exceeded: q=%.5f, speed=%.5f',...
        jointError,max(abs(dq)));
end
if poseError>.003 || angle>deg2rad(1)
    error('Rod tracking guard exceeded: position=%.6f m angle=%.6f rad',...
        poseError,angle);
end
rise=rod(3,4)-path.rod0(3,4);
if rise<-.003 || rise>height+.003
    error('Rod left vertical test envelope');
end
if max(abs(cache.fingers-initialFingers))>.0005
    error('Finger position changed');
end
graspPosition=0;graspRotation=0;
if checkGrasp
    for i=1:2
        idx=(6*i-5):(6*i);
        [tip,~]=dualarm.armGeometry(arms{i},q(idx));
        expected=rod*grasp{i};
        graspPosition=max(graspPosition,...
            norm(tip(1:3,4)-expected(1:3,4)));
        graspRotation=max(graspRotation,norm(dualarm.rotLog(...
            tip(1:3,1:3)*expected(1:3,1:3).')));
    end
    if graspPosition>.002 || graspRotation>deg2rad(1)
        error('Grasp relative pose changed');
    end
end
metrics=struct('joint_error_rad',jointError,'rod_error_m',poseError,...
    'rod_angle_error_rad',angle,'grasp_position_error_m',graspPosition,...
    'grasp_rotation_error_rad',graspRotation,'rise_m',rise);
end
