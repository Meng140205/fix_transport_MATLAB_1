function R=quatToRot(q)
%QUATTOROT Quaternion [x;y;z;w] to a 3x3 rotation matrix.
q=q(:);
if numel(q)~=4 || any(~isfinite(q)) || abs(norm(q)-1)>.01
    error('Invalid unit quaternion');
end
q=q/norm(q);x=q(1);y=q(2);z=q(3);w=q(4);
R=[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w);...
   2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w);...
   2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)];
end
