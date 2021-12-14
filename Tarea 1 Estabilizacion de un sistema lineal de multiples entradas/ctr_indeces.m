function mu = ctr_indeces(A,B)

[n p] = size(B);

C = [];
for i = 0:(n-1)
    C = [C A^i*B];
end

li = [];
mu = zeros(p,1);

for i = 1:(n*p)
    li = [li C(:,i)];
    
    if rank(li) == i
        index = mod(i,p)+1;
        mu(index) = mu(index) + 1;
    end
end

end