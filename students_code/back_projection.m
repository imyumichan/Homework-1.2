function [backCoords,backDescriptors]=back_projection(num_samples,keypoints,descriptors,orientations,locations,vert1,vert2,vert3,camera_params)

perm = randperm(size(keypoints,2)) ;
sel = perm(1:num_samples);
P = camera_params.IntrinsicMatrix.'*[orientations -orientations*locations(:,:,1).'];
Q = P(:,1:3);
q = P(:,4);

orig = -inv(Q)*q; % this corresponds to C
model.coord3d = [];
model.descriptors = [];
for j=1:num_samples
    keypoints_coordinates=keypoints(1:2,:);
    dir=keypoints_coordinates(:,sel(j));
    dir=[dir;1];
    dir=Q\dir;
    [intersect, ~, ~, ~, xcoor] = TriangleRayIntersection(orig, dir, vert1, vert2, vert3, 'planeType', 'one sided');
    xcoor(any(isnan(xcoor)'),:) = [];
    if ~isempty(xcoor)
        model.coord3d=[model.coord3d;xcoor];
        model.descriptors = [model.descriptors,descriptors(:,sel(j))];
    end
end
backCoords = model.coord3d;
backDescriptors = model.descriptors;
end