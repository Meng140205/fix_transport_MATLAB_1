function [tip,J,M,g] = armGeometry(arm,q)
%ARMGEOMETRY FK, [linear;angular] Jacobian, joint mass and gravity terms.
q=q(:);
if numel(q)~=6 || any(~isfinite(q)),error('Expected six finite joints');end
T=eye(4);axes=zeros(3,6);centers=zeros(3,6);count=0;
bodyT=cell(1,numel(arm.parts));bodyCount=zeros(1,numel(arm.parts));
for k=1:numel(arm.parts)
    part=arm.parts(k);
    T=T*part.origin;
    if ~strcmp(part.kind,'fixed')
        count=count+1;
        axes(:,count)=T(1:3,1:3)*part.axis;
        centers(:,count)=T(1:3,4);
        turn=eye(4);
        turn(1:3,1:3)=axisAngle(part.axis,q(count));
        T=T*turn;
    end
    bodyT{k}=T;
    bodyCount(k)=count;
end
tip=T;
J=jacAt(tip(1:3,4),6,axes,centers);
M=zeros(6);g=zeros(6,1);
for k=1:numel(arm.parts)
    if bodyCount(k)==0 || arm.parts(k).mass==0,continue;end
    [M,g]=accumulate(M,g,bodyT{k},bodyCount(k),...
        arm.parts(k),axes,centers);
end
for k=1:numel(arm.fingers)
    finger=arm.fingers(k);
    [M,g]=accumulate(M,g,tip*finger.transform,6,...
        finger,axes,centers);
end
M=(M+M.')/2;
end

function [M,g]=accumulate(M,g,T,count,body,axes,centers)
R=T(1:3,1:3);
point=T(1:3,4)+R*body.com;
J=jacAt(point,count,axes,centers);
Jv=J(1:3,:);Jw=J(4:6,:);
Iw=R*body.inertia*R.';
M=M+body.mass*(Jv.'*Jv)+Jw.'*Iw*Jw;
g=g+Jv.'*[0;0;body.mass*9.81];
end

function J=jacAt(point,count,axes,centers)
J=zeros(6,6);
for j=1:count
    J(1:3,j)=cross(axes(:,j),point-centers(:,j));
    J(4:6,j)=axes(:,j);
end
end

function R=axisAngle(axis,angle)
a=axis/norm(axis);
K=[0 -a(3) a(2);a(3) 0 -a(1);-a(2) a(1) 0];
R=eye(3)+sin(angle)*K+(1-cos(angle))*(K*K);
end
