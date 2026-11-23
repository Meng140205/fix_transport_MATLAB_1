root=fileparts(fileparts(mfilename('fullpath')));
urdf=fullfile(root,'lift_failure_review','actual_robot.urdf');
sdf=fullfile(root,'mpc_readonly_review_2026-11-16','rod_with_pose.sdf');
q=[1.1519853;-2.1270778;2.0490423;-1.4927645;1.5707890;-0.4188037];
rod=eye(4);rod(1:3,4)=[0;.29966;1.2717];
result=preview_nominal_lift(urdf,sdf,[q;q],rod,.30,60);
assert(result.ik_samples==61);
assert(result.nominal_qp_samples==1201);
