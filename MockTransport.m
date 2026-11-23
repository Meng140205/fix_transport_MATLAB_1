classdef MockTransport < handle
    %MOCKTRANSPORT Offline handshake check; never touches ROS.
    properties
        cache
        sent=false
        committed=false
        stopped=false
    end
    methods
        function obj=MockTransport()
            obj.cache=dualarm.MpcStateCache();
            obj.cache.simTime=10;obj.cache.jointStamp=10;obj.cache.rodStamp=10;
            stamp=dualarm.makeStamp(10);
            status=struct('stamp',stamp,'mode',uint8(0),...
                'state_valid',true,'session_id',uint64(0),...
                'ready_plan_id',uint64(0),'active_plan_id',uint64(0));
            obj.cache.statusLeft=status;obj.cache.statusRight=status;
        end
        function sendPlan(obj,sessionId,planId,~,~,~,~)
            obj.sent=true;
            obj.cache.statusLeft.session_id=sessionId;
            obj.cache.statusRight.session_id=sessionId;
            obj.cache.statusLeft.ready_plan_id=planId;
            obj.cache.statusRight.ready_plan_id=planId;
        end
        function sendCommit(obj,~,planId,~,~)
            obj.committed=true;
            obj.cache.statusLeft.mode=uint8(2);
            obj.cache.statusRight.mode=uint8(2);
            obj.cache.statusLeft.active_plan_id=planId;
            obj.cache.statusRight.active_plan_id=planId;
        end
        function stop(obj)
            obj.stopped=true;
            obj.cache.statusLeft.mode=uint8(0);
            obj.cache.statusRight.mode=uint8(0);
            obj.cache.statusLeft.active_plan_id=uint64(0);
            obj.cache.statusRight.active_plan_id=uint64(0);
        end
        function pauseForMessages(~)
        end
    end
end
