function [M,h,J,tip] = armEvaluate(arm,q,dq)
%ARMEVALUATE Joint dynamics for one EC66 using the URDF's rigid bodies.
% The Christoffel velocity term is computed from centered M derivatives.
q=q(:);dq=dq(:);
if numel(q)~=6 || numel(dq)~=6 || ...
        any(~isfinite(q)) || any(~isfinite(dq))
    error('Expected finite six-dimensional q and dq');
end
[tip,J,M,g]=dualarm.armGeometry(arm,q);
c=zeros(6,1);
if any(abs(dq)>1e-12)
    epsilon=1e-5;
    derivative=zeros(6,6,6);
    for k=1:6
        step=zeros(6,1);step(k)=epsilon;
        [~,~,Mp]=dualarm.armGeometry(arm,q+step);
        [~,~,Mm]=dualarm.armGeometry(arm,q-step);
        derivative(:,:,k)=(Mp-Mm)/(2*epsilon);
    end
    for i=1:6
        for j=1:6
            for k=1:6
                gamma=0.5*(derivative(i,j,k)+...
                    derivative(i,k,j)-derivative(j,k,i));
                c(i)=c(i)+gamma*dq(j)*dq(k);
            end
        end
    end
end
h=c+g+arm.damping.*dq;
end
