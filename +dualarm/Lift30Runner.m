classdef Lift30Runner < handle
    %LIFT30RUNNER MATLAB 30 cm cooperative MPC with paired ROS 2 commands.
    properties
        transport
        cache
        arms
        payload
        grasp
        path
        initialFingers
        lower
        upper
        limits
        kp
        kd
        anchorQ
        anchorDq
        anchorTau
        anchorGravity
        residualState=[]
        lastBreakaway=zeros(12,1)
        phaseStart=[]
        motionStart=[]
        reanchored=false
        lastSolve=-Inf
        planId=uint64(0)
        sessionId=uint64(0)
        maxRodError=0
        maxJointError=0
        height=.30
        duration=60
        testMode=false
        logPath
    end
    methods
        function obj=Lift30Runner(urdf,sdf,domainId,commands,testTransport)
            testMode=nargin>=5 && ~isempty(testTransport);
            obj.testMode=testMode;
            if testMode
                if commands && ~isa(testTransport,'MockExecuteTransport')
                    error('Only the offline execution mock may bypass ROS parameters');
                end
                obj.transport=testTransport;
            else
                obj.transport=dualarm.RosTransport(domainId,commands);
            end
            obj.cache=obj.transport.cache;
            deadline=tic;
            while toc(deadline)<30 && ~obj.cache.ready('rod')
                obj.transport.pauseForMessages();
            end
            if ~obj.cache.ready('rod'),error('ROS 2 state streams unavailable');end
            if obj.cache.statusLeft.mode~=0 || obj.cache.statusRight.mode~=0 || ...
                    ~obj.cache.statusLeft.state_valid || ~obj.cache.statusRight.state_valid
                error('Both arms must begin in valid HOLD');
            end
            if max(abs(obj.cache.dq))>.01 || max(abs(obj.cache.rodVelocity))>.005
                error('Arms and rod must settle before MPC initialization');
            end
            obj.initialFingers=obj.cache.fingers;
            obj.arms={dualarm.loadArm(urdf,'left',obj.initialFingers(1:2).'),...
                      dualarm.loadArm(urdf,'right',obj.initialFingers(3:4).')};
            obj.payload=dualarm.rodInertia(sdf);
            obj.path=dualarm.liftReference(obj.arms,obj.cache.q,...
                obj.cache.rod,obj.height,obj.duration);
            obj.grasp=cell(1,2);
            for i=1:2
                idx=(6*i-5):(6*i);
                [~,~,~,tip]=dualarm.armEvaluate(obj.arms{i},...
                    obj.cache.q(idx),obj.cache.dq(idx));
                obj.grasp{i}=obj.cache.rod\tip;
            end
            obj.lower=[obj.arms{1}.lower;obj.arms{2}.lower];
            obj.upper=[obj.arms{1}.upper;obj.arms{2}.upper];
            obj.limits=[obj.arms{1}.effort;obj.arms{2}.effort];
            if commands
                if testMode
                    obj.kp=repmat([100;100;100;30;30;30],2,1);
                    obj.kd=repmat([15;15;15;5;5;5],2,1);
                else
                    [obj.kp,obj.kd]=dualarm.checkControllerParameters(domainId);
                end
            end
        end
        function report=preview(obj,urdf,sdf)
            previous=obj.currentEffort();
            before=obj.cache.simTime;
            report=preview_nominal_lift(urdf,sdf,obj.path.q0,...
                obj.path.rod0,obj.height,obj.duration,previous,obj.initialFingers);
            deadline=tic;
            while toc(deadline)<10
                obj.transport.pauseForMessages();
                if obj.cache.ready('rod') && obj.cache.simTime>before
                    break
                end
            end
            if ~obj.cache.ready('rod') || obj.cache.simTime<=before
                error('State or simulation clock stale after preview');
            end
            shift=norm(obj.cache.rod(1:3,4)-obj.path.rod0(1:3,4));
            if shift>.002,error('Rod drifted >2 mm during preview');end
        end
        function report=execute(obj,urdf,sdf)
            obj.preview(urdf,sdf);
            if obj.cache.statusLeft.mode~=0 || obj.cache.statusRight.mode~=0
                error('Preview must end with both arms in HOLD');
            end
            obj.anchorQ=obj.cache.q;
            obj.anchorDq=obj.cache.dq;
            obj.anchorTau=obj.currentEffort();
            obj.anchorGravity=obj.gravity(obj.anchorQ);
            obj.sessionId=uint64(floor(posixtime(datetime('now'))*1e6));
            stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
            obj.logPath=fullfile(pwd,sprintf('matlab_lift30_%s.csv',stamp));
            file=fopen(obj.logPath,'w');
            if file<0,error('Cannot create trajectory log');end
            closeLog=onCleanup(@()fclose(file)); %#ok<NASGU>
            headers={'sim_time','plan_id','reference_z','actual_z',...
                'rod_error_m','joint_error_rad','rod_angle_error_rad'};
            for field={'actual_tau','q','q_ref','dq','dq_ref',...
                    'residual_bias','breakaway_bias'}
                for side={'L','R'}
                    for joint=1:6
                        headers{end+1}=sprintf('%s_%s%d',...
                            field{1},side{1},joint); %#ok<AGROW>
                    end
                end
            end
            fprintf(file,'%s\n',strjoin(headers,','));
            stopGuard=onCleanup(@()obj.transport.stop()); %#ok<NASGU>
            finish=Inf;lastClock=obj.cache.simTime;clockWall=tic;
            try
                while obj.cache.simTime<finish
                    obj.transport.pauseForMessages();
                    if obj.cache.simTime>lastClock
                        lastClock=obj.cache.simTime;clockWall=tic;
                    elseif toc(clockWall)>15
                        error('Simulation clock stopped');
                    end
                    obj.maybeReanchor();
                    metrics=dualarm.checkLiftGuard(obj.cache,obj.path,...
                        obj.arms,obj.grasp,obj.initialFingers,...
                        obj.motionStart,obj.height,false);
                    obj.maxRodError=max(obj.maxRodError,metrics.rod_error_m);
                    obj.maxJointError=max(obj.maxJointError,metrics.joint_error_rad);
                    if ~isempty(obj.phaseStart) && ...
                            (obj.cache.statusLeft.mode~=2 || obj.cache.statusRight.mode~=2)
                        error('Unexpected MPC dropout');
                    end
                    simNow=obj.cache.simTime;
                    if simNow-obj.lastSolve<.049,continue;end
                    [torque,stamp]=obj.solveCurrent(simNow);
                    obj.transport.pauseForMessages();
                    if ~obj.cache.ready('rod') || obj.cache.simTime-stamp>.08
                        error('Computed plan stale before submission');
                    end
                    simNow=obj.cache.simTime;
                    start=ceil((simNow+.04)*100)/100;
                    if isempty(obj.phaseStart)
                        obj.phaseStart=start;
                        obj.motionStart=start+3;
                        finish=start+3+obj.duration+2;
                    end
                    alpha=dualarm.quintic(min(1,max(0,(start-obj.phaseStart)/2)));
                    q=obj.cache.q;dq=obj.cache.dq;
                    actual=obj.currentEffort();
                    command=dualarm.composeTorque(alpha,torque,...
                        obj.anchorTau,obj.gravity(q),obj.anchorGravity,...
                        obj.kp,obj.kd,obj.anchorQ,q,obj.anchorDq,dq,...
                        actual,obj.limits);
                    obj.planId=obj.planId+uint64(1);
                    dualarm.submitPairedPlan(obj.transport,'rod',obj.sessionId,...
                        obj.planId,stamp,start,start+.12,command);
                    obj.lastSolve=simNow;
                    metrics=dualarm.checkLiftGuard(obj.cache,obj.path,...
                        obj.arms,obj.grasp,obj.initialFingers,...
                        obj.motionStart,obj.height,false);
                    [qref,dqref,reference]=dualarm.evalLiftReference(obj.path,...
                        obj.cache.simTime-obj.motionStart);
                    effort=obj.currentEffort();
                    if isempty(obj.residualState),bias=zeros(12,1);
                    else,bias=obj.residualState.value;end
                    values=[obj.cache.simTime;double(obj.planId);...
                        reference(3,4);obj.cache.rod(3,4);...
                        metrics.rod_error_m;metrics.joint_error_rad;...
                        metrics.rod_angle_error_rad;effort;obj.cache.q;...
                        qref;obj.cache.dq;dqref;bias;obj.lastBreakaway];
                    fields=arrayfun(@(x)sprintf('%.12g',x),values,...
                        'UniformOutput',false);
                    fprintf(file,'%s\n',strjoin(fields,','));
                end
                dualarm.stopPaired(obj.transport);
                holdEnd=obj.cache.simTime+2;
                while obj.cache.simTime<holdEnd
                    obj.transport.pauseForMessages();
                    if obj.cache.statusLeft.mode~=0 || obj.cache.statusRight.mode~=0
                        error('HOLD did not persist');
                    end
                    dualarm.checkLiftGuard(obj.cache,obj.path,obj.arms,...
                        obj.grasp,obj.initialFingers,...
                        obj.motionStart,obj.height,false);
                end
                rise=obj.cache.rod(3,4)-obj.path.rod0(3,4);
                if abs(rise-obj.height)>.002 || ...
                        max(abs(obj.cache.dq))>.01 || ...
                        max(abs(obj.cache.rodVelocity))>.005
                    error('30 cm endpoint not reached or not settled');
                end
                report=struct('result','MATLAB_LIFT_30CM_PASS',...
                    'test_mode',obj.testMode,...
                    'measured_lift_m',rise,'desired_lift_m',obj.height,...
                    'max_rod_error_m',obj.maxRodError,...
                    'max_joint_error_rad',obj.maxJointError,...
                    'plans_sent',double(obj.planId),'trajectory_csv',obj.logPath);
                disp(jsonencode(report));
            catch problem
                try
                    dualarm.stopPaired(obj.transport);
                    fprintf('STOP_CONFIRMED: both controllers in HOLD\n');
                catch stopProblem
                    obj.transport.stop();
                    fprintf(2,'STOP_REQUESTED: %s\n',stopProblem.message);
                end
                fprintf(2,'MATLAB_LIFT_30CM_STOPPED %s\n',problem.message);
                rethrow(problem)
            end
        end
        function maybeReanchor(obj)
            if obj.reanchored || isempty(obj.motionStart) || ...
                    obj.cache.simTime<obj.motionStart
                return
            end
            [obj.path,obj.grasp,report]=dualarm.reanchorLift(obj.path,...
                obj.arms,obj.cache.q,obj.cache.dq,obj.cache.fingers,...
                obj.cache.rod,obj.cache.rodVelocity,...
                obj.cache.jointStamp,obj.cache.rodStamp,...
                obj.cache.simTime,obj.initialFingers);
            obj.motionStart=obj.cache.simTime;
            obj.reanchored=true;
            fprintf('LIFT_REANCHORED %s\n',jsonencode(report));
        end
        function [torque,stamp]=solveCurrent(obj,simNow)
            cache=obj.cache;
            metrics=dualarm.checkLiftGuard(cache,obj.path,obj.arms,...
                obj.grasp,obj.initialFingers,obj.motionStart,obj.height,true); %#ok<NASGU>
            q=cache.q;dq=cache.dq;rod=cache.rod;
            if abs(cache.jointStamp-cache.rodStamp)>.08
                error('Joint/rod timestamp skew exceeds 80 ms');
            end
            if isempty(obj.motionStart),elapsed=-1;
            else,elapsed=cache.jointStamp-obj.motionStart;end
            [qref,dqref]=dualarm.evalLiftReference(obj.path,elapsed);
            active=~isempty(obj.motionStart) && cache.jointStamp>=obj.motionStart;
            [obj.residualState,residual]=dualarm.trackingBiasStep(...
                obj.residualState,qref-q,cache.jointStamp,active);
            [friction,extra]=dualarm.frictionFeedforward(...
                [obj.arms{1}.friction;obj.arms{2}.friction],dqref,dq);
            obj.lastBreakaway=extra;
            model=dualarm.localModel(obj.arms,q,dq,rod,...
                cache.rodVelocity,obj.grasp,obj.payload,friction+residual);
            refs=zeros(11,36);
            for k=0:10
                if isempty(obj.motionStart),t=-1;
                else,t=simNow-obj.motionStart+k*.05;end
                [qr,dqr,pr,vr]=dualarm.evalLiftReference(obj.path,t);
                poseError=[pr(1:3,4)-rod(1:3,4);...
                    dualarm.rotLog(pr(1:3,1:3)*rod(1:3,1:3).')];
                refs(k+1,:)=[qr-q;poseError;dqr;vr].';
            end
            previous=obj.currentEffort();
            result=dualarm.solveMPC(model,q,refs,previous,...
                obj.lower,obj.upper,obj.limits,10,.05);
            torque=result.torque;
            stamp=min(cache.jointStamp,cache.rodStamp);
        end
        function result=gravity(obj,q)
            result=zeros(12,1);
            for i=1:2
                idx=(6*i-5):(6*i);
                [~,h]=dualarm.armEvaluate(obj.arms{i},q(idx),zeros(6,1));
                result(idx)=h;
            end
        end
        function effort=currentEffort(obj)
            effort=[double(obj.cache.statusLeft.command_effort(:));...
                    double(obj.cache.statusRight.command_effort(:))];
        end
    end
end
