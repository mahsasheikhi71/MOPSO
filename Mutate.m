function [y1, y2]=Mutate(x1,x2,Parameter)

y1=x1;
y2=x2;

T=rand;

if T<Parameter.Mut1
    A=randsample(numel(x1),Parameter.NumMut1);
    y1(A)=rand(size(A));    
else
    A=randsample(numel(x2),Parameter.NumMut2);
    y2(A)=1-y2(A);
end

end