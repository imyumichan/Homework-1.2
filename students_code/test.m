
function J = test(intrinsicMatrix,rotations,locations,true_points_2d,points_3d)
syms r1 r2 r3 t1 t2 t3 u v X Y Z 
A = intrinsicMatrix;
r = [r1 r2 r3];
theta = norm(r);
m = [u v]';
mat = [0 -r3 -r2;r3 0 -r1;-r2 r1 0];
R = (eye(3) +(sin(theta)*mat)/theta  + ((1 - cos(theta))*mat*mat)/theta^2)';
M = [X Y Z];
T = [t1 t2 t3];
p = [r1 r2 r3 t1 t2 t3];
M_cam = R*M' + T';
m_uvw = A*M_cam;
m_img = m_uvw./m_uvw(3);
m_img = m_img(1:2);

res = m_img - m;

J = jacobian(res,p);

rotV = rotationMatrixToVector(rotations);
rotV = rotV/norm(rotV);
J = subs(J,r,rotV);
J = subs(J,T,locations);
J = subs(J,M,points_3d);
J = subs(J,m,true_points_2d);

% J = subs(J,A,intrinsicMatrix);

J = double(J);
end