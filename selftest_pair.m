addpath(fileparts(mfilename('fullpath')));
transport=MockTransport();
dualarm.submitPairedPlan(transport,'rod',uint64(123),uint64(1),...
    10,10.05,10.17,zeros(12,1));
assert(transport.sent && transport.committed);
dualarm.stopPaired(transport);
assert(transport.stopped);
fprintf('PAIR_PROTOCOL_PASS\n');
