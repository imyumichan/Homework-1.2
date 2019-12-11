function [err] = ro_calculate(e)
c = 4.685;
c_const = (c^2)/6;
err = zeros(size(e,1));
err = err(1,:);
for i=1:size(e,1)
   if e(i) <= c
       new = c_const * (1 - (1 - (e(i)/c)^2)^3);
%        fprintf(" %f %f \n",e(i),new);
       err(i) = new;
   else
      err(i) = c_const;
   end
end
end