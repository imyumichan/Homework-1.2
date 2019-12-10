function [init_orientation,init_location] = find_initial_pose()
load('sift_model.mat');
load('init_keypoints.mat');
load('init_descriptors.mat');

intrinsicMatrix = [2960.37845 0 0;
    0 2960.37845 0;
    1841.68855 1235.23369  1];

camera_params = cameraParameters('IntrinsicMatrix',intrinsicMatrix);
% camera_params = cameraParameters();
cam_intrinsics=camera_params;

% threshold_ubcmatch = 1.5;
threshold_ransac = 20;
%     Match features between SIFT model and SIFT features from new image
load('init_matches.mat');
% init_sift_matches = vl_ubcmatch(init_descriptors, model.descriptors, threshold_ubcmatch);
% TODO: Estimate camera position for the first image
ransac_iterations = 10000;
max_reproj_err = 10000;

best_inliers=[];

for j=1:ransac_iterations
    rand_sample=init_sift_matches(:,randperm(size(init_sift_matches,2),4));
    detect_world_points=model.coord3d(rand_sample(2,:),:);
    judge_repeat=unique(detect_world_points,'rows'); % make sure to get a sample without repetitions
    while size(detect_world_points,1)~=size(judge_repeat,1)
        rand_sample=init_sift_matches(:,randperm(size(init_sift_matches,2),4));
        detect_world_points=model.coord3d(rand_sample(2,:),:);
        judge_repeat=unique(detect_world_points,'rows');
    end
    detect_image_points=init_keypoints(1:2,rand_sample(1,:))';
    
    [cam_world_orientations,cam_world_locations,inlierIdx,status] ...,
        = estimateWorldCameraPose(detect_image_points, detect_world_points, camera_params, 'MaxReprojectionError', max_reproj_err);
    
    points_thd=model.coord3d(init_sift_matches(2,:),:)';
    points_td = project3d2image(points_thd, cam_intrinsics, cam_world_orientations, cam_world_locations);
    points_otd=init_keypoints(1:2,init_sift_matches(1,:));
    inlier_temp=[];
    inlier_td_temp=[];
    best_inlier_temp=[];
    for k=1:size(points_td,2)
        if ((points_td(:,k)-points_otd(:,k))')*(points_td(:,k)-points_otd(:,k))<threshold_ransac*threshold_ransac
            best_inlier_temp=[best_inlier_temp;k];
        end
    end
    if size(best_inlier_temp,1)>size(best_inliers)
        
        best_inliers=best_inlier_temp;
    end
    
end

inlier=model.coord3d(init_sift_matches(2,best_inliers),:);
inlier_td=init_keypoints(1:2,init_sift_matches(1,best_inliers))';




[init_orientation, init_location] = estimateWorldCameraPose(inlier_td, inlier, camera_params, 'MaxReprojectionError', 4);


end