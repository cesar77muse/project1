function yf=linefit1(y,bp,N)

%bp=[0;bp(:)];
%y=[y(1);y(:);y(length(y))];
lb=length(bp)-1;
yf=[];

for kb=1:lb
    yb2=y(kb+1);
    yb1=y(kb);
    m=(yb2-yb1)/(bp(kb+1)-bp(kb));
    x=(0:1/N:(bp(kb+1)-bp(kb)-1/N))';
    yf=[yf;m.*x+y(kb)];
end
yf=[yf];
    
    
    
    
    