function [kp,kd]=checkControllerParameters(domainId)
%CHECKCONTROLLERPARAMETERS Verify the existing stage3 C++ controllers.
kp=zeros(12,1);kd=zeros(12,1);
for i=1:2
    if i==1,name='/left_arm_hold_controller';
    else,name='/right_arm_hold_controller';end
    client=ros2param(name,DomainID=domainId);
    enabled=get(client,'enable_mpc');
    revision=get(client,'controller_revision');
    if ~enabled || ~strcmp(char(revision),'stage3_hover_v1')
        error('%s must have stage3_hover_v1 and enable_mpc=true',name);
    end
    idx=(6*i-5):(6*i);
    gainsP=get(client,'kp');gainsD=get(client,'kd');
    if numel(gainsP)~=6 || numel(gainsD)~=6
        error('%s controller gains malformed',name);
    end
    kp(idx)=double(gainsP(:));kd(idx)=double(gainsD(:));
end
end
