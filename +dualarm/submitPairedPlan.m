function submitPairedPlan(transport,mode,sessionId,planId,...
    stateStamp,startTime,validUntil,effort)
%SUBMITPAIREDPLAN Propose, await both READY, commit and observe both ACTIVE.
cache=transport.cache;
if ~cache.ready(mode),error('State/status stale before plan');end
if cache.simTime-stateStamp>.08 || startTime-cache.simTime>.15 || ...
        validUntil-startTime>.2 || validUntil<=startTime
    error('Invalid plan timing');
end
transport.sendPlan(sessionId,planId,stateStamp,startTime,validUntil,effort);
wall=tic;
while true
    transport.pauseForMessages();
    if ~cache.ready(mode),transport.stop();error('State/status stale while awaiting READY');end
    left=cache.statusLeft;right=cache.statusRight;
    if left.mode==3 || right.mode==3
        transport.stop();error('Arm reported FAULT while awaiting READY');
    end
    if left.session_id==sessionId && right.session_id==sessionId && ...
            left.ready_plan_id==planId && right.ready_plan_id==planId
        break
    end
    if cache.simTime>=startTime-.01 || toc(wall)>5
        transport.stop();error('Both arms did not prepare before start time');
    end
end
if cache.simTime>=startTime-.005
    transport.stop();error('READY handshake missed start deadline');
end
transport.sendCommit(sessionId,planId,startTime,validUntil);
wall=tic;
while true
    transport.pauseForMessages();
    if ~cache.ready(mode),transport.stop();error('State/status stale while awaiting ACTIVE');end
    left=cache.statusLeft;right=cache.statusRight;
    if left.mode==2 && right.mode==2 && ...
            left.session_id==sessionId && right.session_id==sessionId && ...
            left.active_plan_id==planId && right.active_plan_id==planId
        return
    end
    if left.mode==3 || right.mode==3 || ...
            cache.simTime>startTime+.04 || toc(wall)>5
        transport.stop();error('Paired plan activation was not observed');
    end
end
end
