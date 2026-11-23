function model=localModel(arms,q,dq,rod,rodVelocity,grasp,payload,friction)
%LOCALMODEL Constrained two-arm/rod dynamics, linearized at measured state.
q=q(:);dq=dq(:);friction=friction(:);
if numel(q)~=12 || numel(dq)~=12 || numel(friction)~=12
    error('Expected 12 arm positions, velocities and friction terms');
end
data=cell(1,2);
for i=1:2
    sel=(6*i-5):(6*i);
    [m,h,j,t]=dualarm.armEvaluate(arms{i},q(sel),dq(sel));
    data{i}=struct('M',m,'h',h,'J',j,'tip',t);
end
M=dualarm.blockDiagonal(data{1}.M,data{2}.M);
h=[data{1}.h;data{2}.h]+friction;
if isempty(rod)
    B=eye(12);
    model=struct('F',M\B,'g',M\(-h),'velocity',dq,...
        'Aclosure',[],'error',[],'L',[],'l',[]);
    return
end
if nargin<8 || isempty(grasp) || isempty(payload)
    error('Rod model requires grasp transforms and payload');
end
rodVelocity=rodVelocity(:);
if numel(rodVelocity)~=6,error('Expected six-dimensional rod twist');end
Iw=rod(1:3,1:3)*payload.inertia*rod(1:3,1:3).';
Mo=dualarm.blockDiagonal(payload.mass*eye(3),Iw);
omega=rodVelocity(4:6);
ho=[0;0;payload.mass*9.81;cross(omega,Iw*omega)];
M=dualarm.blockDiagonal(M,Mo);h=[h;ho];
A=zeros(12,18);e=zeros(12,1);AdotV=zeros(12,1);
for i=1:2
    rows=(6*i-5):(6*i);cols=rows;
    expected=rod*grasp{i};tip=data{i}.tip;
    r=expected(1:3,4)-rod(1:3,4);
    G=[eye(3) -dualarm.skew(r);zeros(3) eye(3)];
    A(rows,cols)=data{i}.J;
    A(rows,13:18)=-G;
    e(rows)=[tip(1:3,4)-expected(1:3,4);...
        dualarm.rotLog(tip(1:3,1:3)*expected(1:3,1:3).')];
    qi=q(cols);vi=dq(cols);epsilon=1e-4;
    [~,Jp]=dualarm.armGeometry(arms{i},qi+epsilon*vi);
    [~,Jm]=dualarm.armGeometry(arms{i},qi-epsilon*vi);
    Gdot=zeros(6);Gdot(1:3,4:6)=-dualarm.skew(cross(omega,r));
    AdotV(rows)=((Jp-Jm)/(2*epsilon))*vi-Gdot*rodVelocity;
end
v=[dq;rodVelocity];
rhs=-AdotV-20*e-8*(A*v);
B=[eye(12);zeros(6,12)];
system=[M -A.';A zeros(12)];
if cond(system)>1e12,error('Ill-conditioned closed-chain model');end
solution=system\[[B;zeros(12,12)] [-h;rhs]];
model=struct('F',solution(1:18,1:12),'g',solution(1:18,13),...
    'velocity',v,'Aclosure',A,'error',e,...
    'L',solution(19:30,1:12),'l',solution(19:30,13));
end
