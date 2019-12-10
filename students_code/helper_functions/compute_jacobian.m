function J=compute_jacobian(coords_3d,points_uvw,camera_params,dr1,dr2,dr3)
J=[];
for i=1:size(points_uvw,2)
    M = coords_3d(i,:,:);
    m_tilda = points_uvw(:,i);
    U = m_tilda(1);
    V = m_tilda(2);
    W = m_tilda(3);
    dm_dmtilda = [1/W 0 -U/(W^2);0 1/W -V/(W^2)];
    dmtilda_dcam = camera_params;
    dM_dp = [dr1*M' dr2*M' dr3*M' eye(3)];
    J = [J;dm_dmtilda * dmtilda_dcam * dM_dp];

end

end