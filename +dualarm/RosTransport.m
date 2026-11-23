classdef RosTransport < handle
    %ROSTRANSPORT MATLAB ROS 2 I/O for the existing paired arm controllers.
    properties
        node
        cache
        planPublisher
        commitPublisher
        subscribers
    end
    methods
        function obj=RosTransport(domainId,enableCommands)
            if nargin<1,domainId=0;end
            if nargin<2,enableCommands=false;end
            if exist('ros2node','file')~=2
                error('ROS Toolbox is required');
            end
            obj.cache=dualarm.MpcStateCache();
            obj.node=ros2node('dualarm_matlab_mpc',domainId);
            obj.subscribers=cell(1,5);
            obj.subscribers{1}=ros2subscriber(obj.node,'/clock',...
                'rosgraph_msgs/Clock',@(msg)obj.cache.onClock(msg),...
                'Reliability','besteffort');
            obj.subscribers{2}=ros2subscriber(obj.node,'/joint_states',...
                'sensor_msgs/JointState',@(msg)obj.cache.onJoint(msg),...
                'Reliability','besteffort');
            obj.subscribers{3}=ros2subscriber(obj.node,'/transport_rod/pose',...
                'geometry_msgs/PoseStamped',@(msg)obj.cache.onPose(msg),...
                'Reliability','besteffort');
            obj.subscribers{4}=ros2subscriber(obj.node,...
                '/dualarm_mpc/left_status','dualarm_mpc_msgs/ArmStatus',...
                @(msg)obj.cache.onLeftStatus(msg),'Reliability','reliable');
            obj.subscribers{5}=ros2subscriber(obj.node,...
                '/dualarm_mpc/right_status','dualarm_mpc_msgs/ArmStatus',...
                @(msg)obj.cache.onRightStatus(msg),'Reliability','reliable');
            if enableCommands
                obj.planPublisher=ros2publisher(obj.node,...
                    '/dualarm_mpc/torque_plan','dualarm_mpc_msgs/TorquePlan',...
                    'Reliability','reliable','Depth',4);
                obj.commitPublisher=ros2publisher(obj.node,...
                    '/dualarm_mpc/commit','dualarm_mpc_msgs/PlanCommit',...
                    'Reliability','reliable','Depth',4);
            end
        end
        function sendPlan(obj,sessionId,planId,stateStamp,startTime,validUntil,effort)
            if isempty(obj.planPublisher),error('Command publisher disabled');end
            effort=effort(:);
            if numel(effort)~=12 || any(~isfinite(effort))
                error('Plan requires 12 finite absolute torques');
            end
            msg=ros2message(obj.planPublisher);
            msg.session_id=uint64(sessionId);
            msg.plan_id=uint64(planId);
            msg.state_stamp=dualarm.makeStamp(stateStamp);
            msg.start_time=dualarm.makeStamp(startTime);
            msg.valid_until=dualarm.makeStamp(validUntil);
            msg.effort=double(effort);
            send(obj.planPublisher,msg);
        end
        function sendCommit(obj,sessionId,planId,startTime,validUntil)
            if isempty(obj.commitPublisher),error('Command publisher disabled');end
            msg=ros2message(obj.commitPublisher);
            msg.session_id=uint64(sessionId);
            msg.plan_id=uint64(planId);
            msg.start_time=dualarm.makeStamp(startTime);
            msg.valid_until=dualarm.makeStamp(validUntil);
            msg.run=true;
            send(obj.commitPublisher,msg);
        end
        function stop(obj)
            if isempty(obj.commitPublisher),return;end
            msg=ros2message(obj.commitPublisher);
            msg.session_id=uint64(0);
            msg.plan_id=uint64(0);
            msg.run=false;
            send(obj.commitPublisher,msg);
        end
        function pauseForMessages(~)
            pause(.001);
        end
    end
end
