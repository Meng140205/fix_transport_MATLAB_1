classdef MockLiftTransport < handle
    %MOCKLIFTTRANSPORT Read-only simulated state stream for MATLAB tests.
    properties
        cache
    end
    methods
        function obj=MockLiftTransport(q,rod,effort)
            obj.cache=dualarm.MpcStateCache();
            obj.cache.simTime=10;
            obj.cache.jointStamp=10;
            obj.cache.q=q(:);
            obj.cache.dq=zeros(12,1);
            obj.cache.fingers=ones(4,1)*.01;
            obj.cache.rodStamp=10;
            obj.cache.rod=rod;
            obj.cache.rodVelocity=zeros(6,1);
            stamp=dualarm.makeStamp(10);
            status=struct('stamp',stamp,'mode',uint8(0),...
                'state_valid',true,'session_id',uint64(0),...
                'ready_plan_id',uint64(0),'active_plan_id',uint64(0),...
                'torque_limited',false,'command_effort',zeros(6,1));
            obj.cache.statusLeft=status;
            obj.cache.statusRight=status;
            obj.cache.statusLeft.command_effort=effort(1:6);
            obj.cache.statusRight.command_effort=effort(7:12);
        end
        function pauseForMessages(obj)
            obj.cache.simTime=obj.cache.simTime+.05;
            obj.cache.jointStamp=obj.cache.simTime;
            obj.cache.rodStamp=obj.cache.simTime;
            stamp=dualarm.makeStamp(obj.cache.simTime);
            obj.cache.statusLeft.stamp=stamp;
            obj.cache.statusRight.stamp=stamp;
        end
    end
end
