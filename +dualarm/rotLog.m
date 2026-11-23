function v=rotLog(R)
%ROTLOG Rotation matrix logarithm as a 3-vector, for local pose errors.
if ~isequal(size(R),[3 3]),error('Expected 3x3 rotation');end
c=max(-1,min(1,(trace(R)-1)/2));
theta=acos(c);
vee=[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
if theta<1e-7
    v=vee/2;
elseif abs(pi-theta)<1e-5
    [U,~,~]=svd(R+eye(3));
    axis=U(:,1);axis=axis/norm(axis);
    if dot(axis,vee)<0,axis=-axis;end
    v=theta*axis;
else
    v=theta/(2*sin(theta))*vee;
end
end
