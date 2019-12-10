function [err,W] = ro_calculate(e,sigma)
c = 4.685;
W = zeros(size(e,1),size(e,1));
err = zeros(size(e,1));
err = err(1,:);
for i=1:size(e,1)
   
   c_const = (c^2)/6;
   if e(i) <=c
       new =  c_const * ( 1 - (1 - (e(i)/c)^2)^3);
%        fprintf(" %f %f \n",e(i),new);
       err(i) = new;
      
   else
      err(i) = c_const;
   end
   e_i=e(i)/sigma;
   if e_i < c
       W(i,i) = (1 - (e_i/c)^2)^2;
   else
       W(i,i) = 0;
   end
     
end



end