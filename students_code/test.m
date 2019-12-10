% function [jj1,jj2,jj3] = test(params)
syms r1 r2 r3 t1 t2 t3 M_1 v v_x theta
r = [r1 r2 r3];
theta = norm(r);
r = r/theta;
mat = [0 -r3 -r2;r3 0 -r1;-r2 r1 0];
R = cos(theta)*eye(3) + (1-cos(theta))*(r*r') + sin(theta)*mat;

jj1 = diff(R,r1);
% jj1 = subs(jj1,r1,params(1));
% jj1 = subs(jj1,r2,params(2));
% jj1 = double(subs(jj1,r3,params(3)));
% 
% jj2 = diff(R,r2);
% jj2 = subs(jj2,r1,params(1));
% jj2 = subs(jj2,r2,params(2));
% jj2 = double(subs(jj2,r3,params(3)));
% 
% jj3 = diff(R,r3);
% jj3 = subs(jj3,r1,params(1));
% jj3 = subs(jj3,r2,params(2));
% jj3 = double(subs(jj3,r3,params(3)));
%     
% end
