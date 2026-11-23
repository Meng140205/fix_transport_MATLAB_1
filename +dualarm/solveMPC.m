function result=solveMPC(model,q,reference,previous,lower,upper,limits,N,dt)
%SOLVEMPC Receding-horizon QP with exact state elimination.
% Same local model, weights and bounds as Python tracking_qp.py.
% Uses MATLAB quadprog if the unconstrained minimizer violates constraints.
if nargin<8 || isempty(N),N=10;end
if nargin<9 || isempty(dt),dt=.05;end
started=tic;
q=q(:);previous=previous(:);lower=lower(:);upper=upper(:);limits=limits(:);
n=numel(model.velocity);nx=2*n;nu=12;
if ~ismember(n,[12 18]) || numel(q)~=12 || numel(previous)~=12 || ...
        numel(lower)~=12 || numel(upper)~=12 || numel(limits)~=12
    error('Inconsistent MPC dimensions');
end
F=model.F;g=model.g(:);v=model.velocity(:);
Ad=[eye(n) dt*eye(n);zeros(n) eye(n)];
Bd=[.5*dt^2*F;dt*F];affine=[.5*dt^2*g;dt*g];
if n==12
    weights=[ones(12,1)*2000;ones(12,1)*30];
else
    weights=[ones(12,1)*200;ones(3,1)*1e5;ones(3,1)*3e4;...
             ones(12,1)*20;ones(3,1)*600;ones(3,1)*100];
end
Qbase=diag(weights);
if isequal(size(reference),[n 1])
    refs=repmat([reference(:);zeros(n,1)],1,N+1);
elseif isequal(size(reference),[N+1 nx])
    refs=reference.';
elseif isequal(size(reference),[nx N+1])
    refs=reference;
else
    error('Reference must be n-vector or (N+1)-by-2n trajectory');
end
if any(~isfinite(refs(:))),error('Invalid reference');end
Q=Qbase;
linearClosure=zeros(nx,1);
if ~isempty(model.Aclosure)
    closure=[model.Aclosure zeros(12,n)];
    Q=Q+1e4*(closure.'*closure);
    linearClosure=2e4*closure.'*model.error(:);
end
R=diag(.5./limits.^2);
rlinear=-R*previous;
if ~isempty(model.L)
    R=R+1e-4*(model.L.'*model.L);
    rlinear=rlinear+1e-4*model.L.'*model.l(:);
end
S=zeros((N+1)*nx,N*nu);d=zeros((N+1)*nx,1);
Sk=zeros(nx,N*nu);dk=[zeros(n,1);v];
for k=0:N
    row=k*nx+(1:nx);
    S(row,:)=Sk;d(row)=dk;
    if k<N
        Sk=Ad*Sk;
        col=k*nu+(1:nu);
        Sk(:,col)=Sk(:,col)+Bd;
        dk=Ad*dk+affine;
    end
end
H=zeros(N*nu);f=zeros(N*nu,1);
for k=0:N
    row=k*nx+(1:nx);Srow=S(row,:);drow=d(row);
    stateLinear=-2*Qbase*refs(:,k+1)+linearClosure;
    H=H+2*Srow.'*Q*Srow;
    f=f+Srow.'*(2*Q*drow+stateLinear);
end
for k=0:N-1
    col=k*nu+(1:nu);
    H(col,col)=H(col,col)+2*R;
    f(col)=f(col)+2*rlinear;
end
H=(H+H.')/2;
qlo=max(lower+.01-q,-.15);qhi=min(upper-.01-q,.15);
positionLo=qlo;positionHi=qhi;
velocityLo=-ones(12,1)*.20;velocityHi=-velocityLo;
if n==18
    positionLo=[positionLo;-ones(3,1)*.03;-ones(3,1)*.10];
    positionHi=[positionHi; ones(3,1)*.03; ones(3,1)*.10];
    velocityLo=[velocityLo;-ones(3,1)*.05;-ones(3,1)*.10];
    velocityHi=-velocityLo;
end
stateLo=[positionLo;velocityLo];stateHi=[positionHi;velocityHi];
x0=[zeros(n,1);v];
if any(x0<stateLo-1e-9) || any(x0>stateHi+1e-9)
    error('Initial state is outside MPC trust/speed bounds');
end
stateLower=repmat(stateLo,N+1,1);
stateUpper=repmat(stateHi,N+1,1);
D=eye(N*nu);
for k=2:N
    row=(k-1)*nu+(1:nu);
    D(row,row-nu)=-eye(nu);
end
slewLower=-ones(N*nu,1)*18*dt;
slewUpper=-slewLower;
slewLower(1:nu)=slewLower(1:nu)+previous;
slewUpper(1:nu)=slewUpper(1:nu)+previous;
Aineq=[S;-S;D;-D];
bineq=[stateUpper-d;d-stateLower;slewUpper;-slewLower];
lb=repmat(-limits,N,1);ub=repmat(limits,N,1);
unconstrained=-(H\f);
if all(isfinite(unconstrained)) && ...
        max([Aineq*unconstrained-bineq;lb-unconstrained;unconstrained-ub])<=1e-9
    U=unconstrained;method='feasible_unconstrained_optimum';iterations=0;
else
    options=optimoptions('quadprog','Algorithm','interior-point-convex',...
        'Display','off','MaxIterations',10000);
    [U,~,exitflag,output]=quadprog(H,f,Aineq,bineq,[],[],lb,ub,[],options);
    if exitflag<=0 || isempty(U) || any(~isfinite(U))
        error('quadprog rejected constrained MPC problem');
    end
    method='quadprog';iterations=output.iterations;
end
violation=max([Aineq*U-bineq;lb-U;U-ub;0]);
if violation>2e-4,error('QP residual exceeds limit');end
result=struct('torque',U(1:nu),'solution',U,'method',method,...
    'iterations',iterations,'residual',violation,'elapsed',toc(started),...
    'predictedState',reshape(S*U+d,nx,N+1));
end
