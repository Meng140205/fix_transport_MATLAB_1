function arm = loadArm(urdfFile, side, fingerPosition)
%LOADARM Read one EC66 chain and its two frozen gripper fingers from URDF.
% Uses base MATLAB XML tools; Robotics System Toolbox is not required.
arguments
    urdfFile (1,:) char
    side (1,:) char
    fingerPosition (1,2) double = [0.01 0.01]
end
if ~strcmp(side,'left') && ~strcmp(side,'right')
    error('side must be left or right');
end
doc = xmlread(urdfFile);
root = doc.getDocumentElement;
links = containers.Map('KeyType','char','ValueType','any');
joints = containers.Map('KeyType','char','ValueType','any');
linkNodes = root.getElementsByTagName('link');
for i = 0:linkNodes.getLength-1
    link = linkNodes.item(i);
    links(char(link.getAttribute('name'))) = link;
end
jointNodes = root.getElementsByTagName('joint');
for i = 0:jointNodes.getLength-1
    joint = jointNodes.item(i);
    child = first(joint,'child');
    if isempty(child),continue;end  % Ignore nested ros2_control interface entries.
    joints(char(child.getAttribute('link'))) = joint;
end
childName = [side '_pgc140_base_link'];
chain = {};
while ~strcmp(childName,'world')
    if ~isKey(joints,childName)
        error('Disconnected URDF at %s',childName);
    end
    joint = joints(childName);
    chain{end+1} = {joint,links(childName)}; %#ok<AGROW>
    childName = char(first(joint,'parent').getAttribute('link'));
end
chain = fliplr(chain);
empty = struct('name',{},'kind',{},'origin',{},'axis',{},...
    'mass',{},'com',{},'inertia',{});
parts = empty;
names = cell(1,6);
lower = zeros(6,1);upper = zeros(6,1);
effort = zeros(6,1);velocity = zeros(6,1);
damping = zeros(6,1);friction = zeros(6,1);
count = 0;
for i = 1:numel(chain)
    joint = chain{i}{1};link = chain{i}{2};
    kind = char(joint.getAttribute('type'));
    if ~strcmp(kind,'fixed') && ~strcmp(kind,'revolute') && ~strcmp(kind,'continuous')
        error('Unsupported arm joint type %s',kind);
    end
    axis = [1;0;0];
    axisNode = first(joint,'axis');
    if ~isempty(axisNode)
        axis = vec(char(axisNode.getAttribute('xyz')));
        axis = axis/norm(axis);
    end
    [mass,com,inertia] = inertial(link);
    part = struct('name',char(joint.getAttribute('name')),...
        'kind',kind,'origin',origin(first(joint,'origin')),...
        'axis',axis,'mass',mass,'com',com,'inertia',inertia);
    parts(end+1) = part; %#ok<AGROW>
    if ~strcmp(kind,'fixed')
        count = count+1;
        if count>6,error('More than six arm joints');end
        names{count} = part.name;
        limit = first(joint,'limit');
        lower(count) = number(limit,'lower',-2*pi);
        upper(count) = number(limit,'upper',2*pi);
        effort(count) = number(limit,'effort',NaN);
        velocity(count) = number(limit,'velocity',NaN);
        dyn = first(joint,'dynamics');
        damping(count) = number(dyn,'damping',0);
        friction(count) = number(dyn,'friction',0);
    end
end
expected = arrayfun(@(i)sprintf('%s_joint%d',side,i),1:6,...
    'UniformOutput',false);
if count~=6 || ~isequal(names,expected)
    error('Unexpected arm joint order');
end
fingers = struct('transform',{},'mass',{},'com',{},'inertia',{});
for i = 1:2
    linkName = sprintf('%s_pgc140_finger%d_link',side,i);
    joint = joints(linkName);
    jointOrigin = origin(first(joint,'origin'));
    axis = vec(char(first(joint,'axis').getAttribute('xyz')));
    jointOrigin(1:3,4) = jointOrigin(1:3,4)+...
        jointOrigin(1:3,1:3)*(axis*fingerPosition(i));
    [mass,com,inertia] = inertial(links(linkName));
    fingers(end+1) = struct('transform',jointOrigin,...
        'mass',mass,'com',com,'inertia',inertia); %#ok<AGROW>
end
arm = struct('side',side,'parts',parts,'fingers',fingers,...
    'names',{names},'lower',lower,'upper',upper,'effort',effort,...
    'velocity',velocity,'damping',damping,'friction',friction);
end

function value = first(node,tag)
value = [];
if isempty(node),return;end
found = node.getElementsByTagName(tag);
if found.getLength>0,value=found.item(0);end
end

function value = number(node,key,fallback)
value=fallback;
if ~isempty(node) && node.hasAttribute(key)
    value=str2double(char(node.getAttribute(key)));
end
end

function out = vec(text)
out=sscanf(text,'%f');
if numel(out)~=3,error('Expected 3-vector: %s',text);end
end

function transform = origin(node)
transform=eye(4);
if isempty(node),return;end
xyz=vecOrZero(char(node.getAttribute('xyz')));
rpy=vecOrZero(char(node.getAttribute('rpy')));
cr=cos(rpy(1));sr=sin(rpy(1));
cp=cos(rpy(2));sp=sin(rpy(2));
cy=cos(rpy(3));sy=sin(rpy(3));
rx=[1 0 0;0 cr -sr;0 sr cr];
ry=[cp 0 sp;0 1 0;-sp 0 cp];
rz=[cy -sy 0;sy cy 0;0 0 1];
transform(1:3,1:3)=rz*ry*rx;
transform(1:3,4)=xyz;
end

function out = vecOrZero(text)
if isempty(text),out=zeros(3,1);else,out=vec(text);end
end

function [mass,com,tensor] = inertial(link)
mass=0;com=zeros(3,1);tensor=zeros(3);
inert = first(link,'inertial');
if isempty(inert),return;end
mass = number(first(inert,'mass'),'value',0);
ori = origin(first(inert,'origin'));
com = ori(1:3,4);
n = first(inert,'inertia');
xx=number(n,'ixx',0);xy=number(n,'ixy',0);
xz=number(n,'ixz',0);yy=number(n,'iyy',0);
yz=number(n,'iyz',0);zz=number(n,'izz',0);
local=[xx xy xz;xy yy yz;xz yz zz];
R=ori(1:3,1:3);tensor=R*local*R.';
if mass<0 || min(eig(tensor)) < -1e-10
    error('Invalid inertia for %s',char(link.getAttribute('name')));
end
end
