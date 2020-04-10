function [T] = caculate_poly(coff, t, k)
%coff is the coefficient of poly, with the order from 0 to highest order
%t is the time or the variable of poly. k is the requested derivative
%Init:
D = zeros(size(coff,1),1);
for i=1:size(coff,1)
    D(i) = i-1;
end

T = 0;

%Derivative:
for j=1:k
    for i=1:size(coff,1)
        coff(i) = coff(i) * D(i);

        if D(i) > 0
            D(i) = D(i) - 1;
        end
    end
end
coff

%caculate

for i=1:size(coff,1)
T = T + coff(i) * t^D(i);
end

end

