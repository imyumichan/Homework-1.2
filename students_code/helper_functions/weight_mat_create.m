function W = weight_mat_create(e)

W = zeros(size(e,1));
c = 4.685;
sigma = 1.48257968 * mad(e);
e = e./sigma;
for i = 1:size(e,1)

    if e(i) < c
        W(i,i) = (1 - e(i)^2/c^2)^2;
    else
        W(i,i) = 0;
    end
    
    
end

end