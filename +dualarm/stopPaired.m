function stopPaired(transport)
%STOPPAIRED Request both controllers to HOLD and verify their status.
transport.stop();
wall=tic;start=transport.cache.simTime;
while toc(wall)<5
    transport.pauseForMessages();
    left=transport.cache.statusLeft;
    right=transport.cache.statusRight;
    if ~isempty(left) && ~isempty(right) && ...
            left.mode==0 && right.mode==0 && ...
            left.active_plan_id==0 && right.active_plan_id==0
        return
    end
    if transport.cache.simTime>start+.35
        break
    end
end
error('HOLD not confirmed after stop request; pause Gazebo');
end
