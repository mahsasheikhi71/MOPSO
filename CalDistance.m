function Z=CalDistance(X,Dis)

Z=0;
for i=1:size(X,1)-1
    Z=Z+Dis(X(i),X(i+1));
end


end