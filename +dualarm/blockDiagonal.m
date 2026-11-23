function out=blockDiagonal(varargin)
%BLOCKDIAGONAL Assemble numeric matrices without relying on path blkdiag.m.
rows=0;cols=0;
for i=1:nargin
    rows=rows+size(varargin{i},1);
    cols=cols+size(varargin{i},2);
end
out=zeros(rows,cols);
r=0;c=0;
for i=1:nargin
    value=varargin{i};
    nr=size(value,1);nc=size(value,2);
    out(r+(1:nr),c+(1:nc))=value;
    r=r+nr;c=c+nc;
end
end
