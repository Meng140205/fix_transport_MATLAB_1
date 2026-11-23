classdef MpcStateCache < handle
    %MPCSTATECACHE Latest time-aligned ROS 2 joint, rod and controller states.
    properties
        simTime=-1
        jointStamp=-1
        q=zeros(12,1)
        dq=zeros(12,1)
        fingers=zeros(4,1)
        rodStamp=-1
        rod=eye(4)
        rodVelocity=zeros(6,1)
        previousRodStamp=-1
        previousRod=eye(4)
        statusLeft=[]
        statusRight=[]
    end
    methods
        function onClock(obj,msg)
            t=dualarm.stampSeconds(msg.clock);
            if obj.simTime>0 && t<obj.simTime
                error('Simulation clock reset; restart MATLAB controller');
            end
            obj.simTime=t;
        end
        function onJoint(obj,msg)
            names=cellstr(msg.name);
            wanted=[arrayfun(@(j)sprintf('left_joint%d',j),1:6,'UniformOutput',false),...
                    arrayfun(@(j)sprintf('right_joint%d',j),1:6,'UniformOutput',false)];
            fingerNames={'left_pgc140_finger1_joint','left_pgc140_finger2_joint',...
                'right_pgc140_finger1_joint','right_pgc140_finger2_joint'};
            idx=zeros(12,1);fi=zeros(4,1);
            for j=1:12
                match=find(strcmp(names,wanted{j}),1);
                if isempty(match),return;end
                idx(j)=match;
            end
            for j=1:4
                match=find(strcmp(names,fingerNames{j}),1);
                if isempty(match),return;end
                fi(j)=match;
            end
            q=double(msg.position(idx));dq=double(msg.velocity(idx));
            fingers=double(msg.position(fi));
            if any(~isfinite([q(:);dq(:);fingers(:)]))
                error('Nonfinite joint-state message');
            end
            obj.jointStamp=dualarm.stampSeconds(msg.header.stamp);
            obj.q=q(:);obj.dq=dq(:);obj.fingers=fingers(:);
        end
        function onPose(obj,msg)
            if ~any(strcmp(char(msg.header.frame_id),{'default','world'}))
                error('Unexpected rod world frame');
            end
            p=msg.pose.position;o=msg.pose.orientation;
            T=eye(4);
            T(1:3,4)=[double(p.x);double(p.y);double(p.z)];
            T(1:3,1:3)=dualarm.quatToRot([o.x;o.y;o.z;o.w]);
            stamp=dualarm.stampSeconds(msg.header.stamp);
            if obj.rodStamp>=0
                dt=stamp-obj.rodStamp;
                if dt<0,error('Rod pose time reset');end
                if dt>.001 && dt<.2
                    linear=(T(1:3,4)-obj.rod(1:3,4))/dt;
                    angular=dualarm.rotLog(T(1:3,1:3)*obj.rod(1:3,1:3).')/dt;
                    obj.rodVelocity=.3*[linear;angular]+.7*obj.rodVelocity;
                end
            end
            obj.previousRodStamp=obj.rodStamp;
            obj.previousRod=obj.rod;
            obj.rodStamp=stamp;obj.rod=T;
        end
        function onLeftStatus(obj,msg),obj.statusLeft=msg;end
        function onRightStatus(obj,msg),obj.statusRight=msg;end
        function tf=ready(obj,mode)
            tf=obj.simTime>0 && obj.jointStamp>=0 && ...
                ~isempty(obj.statusLeft) && ~isempty(obj.statusRight);
            if ~tf,return;end
            tf=(obj.simTime-obj.jointStamp<=.15) && ...
               (obj.simTime-dualarm.stampSeconds(obj.statusLeft.stamp)<=.15) && ...
               (obj.simTime-dualarm.stampSeconds(obj.statusRight.stamp)<=.15);
            if strcmp(mode,'rod')
                tf=tf && obj.rodStamp>=0 && obj.simTime-obj.rodStamp<=.15;
            end
        end
    end
end
