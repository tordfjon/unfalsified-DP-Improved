function out = isstable(a)
result = 1;
[rad,col] = size(a);
if rad==col
    poles = eig(a);
    for i = 1:rad
        if real(poles(i))>0
            result=0;
        end
    end
else 
    error('A matrix from realization must be square. Ref: FRG')
end

out = result;
end