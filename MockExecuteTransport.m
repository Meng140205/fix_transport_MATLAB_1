classdef MockExecuteTransport < handle
    %MOCKEXECUTETRANSPORT Perfect-tracking offline plan/protocol simulator.
    properties
        cache
        runner=[]
        pending=[]
        stopped=false
        sent=0
    end
    methods
        function obj=MockExecuteTransport(q,rod,effort)
            obj.cache=dualarm.MpcStateCache();
            obj.cache.simTime=10;obj.cache.jointStamp=10;obj.cache.rodStamp=10;
            obj.cache.q=q(:);obj.cache.dq=zeros(12,1);
            obj.cache.fingers=ones(4,1)*.01;
            obj.cache.rod=rod;obj.cache.rodVelocity=zeros(6,1);
            stamp=dualarm.makeStamp(10);
            s=struct('stamp',stamp,'mode',uint8(0),...
                'state_valid',true,'session_id',uint64(0),...
                'ready_plan_id',uint64(0),'active_plan_id',uint64(0),...
                'torque_limited',false,'command_effort',zeros(6,1));
            obj.cache.statusLeft=s;obj.cache.statusRight=s;
            obj.cache.statusLeft.command_effort=effort(1:6);
            obj.cache.statusRight.command_effort=effort(7:12);
        end
        function pauseForMessages(obj)
            obj.cache.simTime=obj.cache.simTime+.01;
            now=obj.cache.simTime;
            if ~isempty(obj.runner) && ~isempty(obj.runner.motionStart) && ...
                    now>=obj.runner.motionStart
                [q,dq,rod,velocity]=dualarm.evalLiftReference(...
                    obj.runner.path,now-obj.runner.motionStart);
                obj.cache.q=q;obj.cache.dq=dq;
                obj.cache.rod=rod;obj.cache.rodVelocity=velocity;
            end
            obj.cache.jointStamp=now;obj.cache.rodStamp=now;
            stamp=dualarm.makeStamp(now);
            obj.cache.statusLeft.stamp=stamp;
            obj.cache.statusRight.stamp=stamp;
            if ~isempty(obj.pending) && now>=obj.pending.start && ...
                    obj.pending.committed
                obj.cache.statusLeft.mode=uint8(2);
                obj.cache.statusRight.mode=uint8(2);
                obj.cache.statusLeft.active_plan_id=obj.pending.planId;
                obj.cache.statusRight.active_plan_id=obj.pending.planId;
                obj.cache.statusLeft.command_effort=obj.pending.effort(1:6);
                obj.cache.statusRight.command_effort=obj.pending.effort(7:12);
            end
        end
        function sendPlan(obj,sessionId,planId,~,startTime,~,effort)
            obj.sent=obj.sent+1;
            obj.pending=struct('session',sessionId,'planId',planId,...
                'start',startTime,'effort',effort(:),'committed',false);
            obj.cache.statusLeft.session_id=sessionId;
            obj.cache.statusRight.session_id=sessionId;
            obj.cache.statusLeft.ready_plan_id=planId;
            obj.cache.statusRight.ready_plan_id=planId;
        end
        function sendCommit(obj,~,planId,~,~)
            if isempty(obj.pending) || obj.pending.planId~=planId
                error('Mock received unmatched commit');
            end
            obj.pending.committed=true;
        end
        function stop(obj)
            obj.stopped=true;
            obj.cache.statusLeft.mode=uint8(0);
            obj.cache.statusRight.mode=uint8(0);
            obj.cache.statusLeft.active_plan_id=uint64(0);
            obj.cache.statusRight.active_plan_id=uint64(0);
            obj.pending=[];
        end
    end
end
