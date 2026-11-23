function payload=rodInertia(sdfFile)
%RODINERTIA Read rod mass and COM-frame inertia from the active SDF.
doc=xmlread(sdfFile);
models=doc.getElementsByTagName('model');
model=[];
for k=0:models.getLength-1
    candidate=models.item(k);
    if strcmp(char(candidate.getAttribute('name')),'transport_rod')
        model=candidate;break
    end
end
if isempty(model),error('transport_rod model missing');end
links=model.getElementsByTagName('link');
if links.getLength~=1,error('Expected one rod link');end
inert=links.item(0).getElementsByTagName('inertial').item(0);
pose=inert.getElementsByTagName('pose');
if pose.getLength>0
    values=sscanf(char(pose.item(0).getTextContent),'%f');
    if any(abs(values)>1e-9)
        error('Rod inertia origin must coincide with model origin');
    end
end
mass=str2double(char(inert.getElementsByTagName('mass').item(0).getTextContent));
n=inert.getElementsByTagName('inertia').item(0);
xx=getnum(n,'ixx');xy=getnum(n,'ixy');xz=getnum(n,'ixz');
yy=getnum(n,'iyy');yz=getnum(n,'iyz');zz=getnum(n,'izz');
I=[xx xy xz;xy yy yz;xz yz zz];
if mass<=0 || min(eig(I))<=0,error('Invalid rod inertia');end
payload=struct('mass',mass,'inertia',I);
end

function out=getnum(node,tag)
out=str2double(char(node.getElementsByTagName(tag).item(0).getTextContent));
end
