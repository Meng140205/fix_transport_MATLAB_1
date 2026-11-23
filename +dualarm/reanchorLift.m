function [path,grasp,report] = reanchorLift(path,arms,q,dq,fingers,...
    rod,rodVelocity,jointStamp,poseStamp,simTime,initialFingers)
%REANCHORLIFT Recapture settled rod origin at the onset of the lift.
q=q(:);dq=dq(:);fingers=fingers(:);
rodVelocity=rodVelocity(:);initialFingers=initialFingers(:);
old=path.rod0;
shift=norm(rod(1:3,4)-old(1:3,4));
angle=norm(dualarm.rotLog(rod(1:3,1:3)*old(1:3,1:3).'));
jointShift=max(abs(q-path.q0));
if abs(jointStamp-poseStamp)>.08 || shift>.002 || ...
        angle>deg2rad(.5) || jointShift>.001 || ...
        max(abs(rodVelocity(1:3)))>.0002 || ...
        max(abs(rodVelocity(4:6)))>.005 || ...
        max(abs(fingers-initialFingers))>.0005
    error('Pre-lift state changed beyond reanchor bounds');
end
path.rod0=rod;
grasp=cell(1,2);
for i=1:2
    sel=(6*i-5):(6*i);
    [~,~,~,tip]=dualarm.armEvaluate(arms{i},q(sel),dq(sel));
    grasp{i}=rod\tip;
end
report=struct('sim_time',simTime,'old_reference_z_m',old(3,4),...
    'settled_start_z_m',rod(3,4),'shift_m',shift,...
    'orientation_shift_deg',rad2deg(angle),...
    'max_joint_shift_rad',jointShift);
end
