clear
clc
close all
addpath('helper_functions')
addpath(genpath('../../vlfeat-0.9.21/'))

%% Setup
% path to the images folder
path_img_dir = '../../data/tracking/validation/img';
% path to object ply file
object_path = '../../data/teabox.ply';
% path to results folder
results_path = '../../data/tracking/valid/results';

% Read the object's geometry 
% Here vertices correspond to object's corners and faces are triangles
[vertices, faces] = read_ply(object_path);
faces=faces+1;

% Create directory for results
if ~exist(results_path,'dir') 
    mkdir(results_path); 
end

% Load Ground Truth camera poses for the validation sequence
% Camera orientations and locations in the world coordinate system
load('gt_valid.mat')

% TODO: setup camera parameters (camera_params) using cameraParameters()
intrinsicMatrix = [2960.37845 0 0;
                   0 2960.37845 0;
                   1841.68855 1235.23369  1];
camera_params = cameraParameters('IntrinsicMatrix',intrinsicMatrix);

%% Get all filenames in images folder

FolderInfo = dir(fullfile(path_img_dir, '*.JPG'));
Filenames = fullfile(path_img_dir, {FolderInfo.name} );
num_files = length(Filenames);

% Place predicted camera orientations and locations in the world coordinate system for all images here
cam_in_world_orientations = zeros(3,3,num_files);
cam_in_world_locations = zeros(1,3,num_files);

%% Detect SIFT keypoints in all images

% You will need vl_sift() and vl_ubcmatch() functions
% download vlfeat (http://www.vlfeat.org/download.html) and unzip it somewhere
% Don't forget to add vlfeat folder to MATLAB path

% Place SIFT keypoints and corresponding descriptors for all images here
keypoints = cell(num_files,1); 
descriptors = cell(num_files,1); 

% for i=1:length(Filenames)
%     fprintf('Calculating sift features for image: %d \n', i)
%     
% %    TODO: Prepare the image (img) for vl_sift() function
%     img = single(rgb2gray(imread(char(Filenames(i)))));
%     [keypoints{i}, descriptors{i}] = vl_sift(img) ;
% end

% Save sift features and descriptors and load them when you rerun the code to save time
% save('sift_descriptors.mat', 'descriptors')
% save('sift_keypoints.mat', 'keypoints')

load('sift_descriptors.mat');
load('sift_keypoints.mat');

%% Initialization: Compute camera pose for the first image

% As the initialization step for the tracking
% we need to compute the camera pose for the first image 
% The first image and it's camera pose will be our initial frame and initial camera pose for the tracking process

% You can use estimateWorldCameraPose() function or your own implementation
% of the PnP+RANSAC from the previous tasks

% You can get correspondences for PnP+RANSAC either using your SIFT model from the previous tasks
% or by manually annotating corners (e.g. with mark_images() function)


% TODO: Estimate camera position for the first image
% [init_orientation, init_location] = estimateWorldCameraPose(image_points, world_points, camera_params, 'MaxReprojectionError', 4);
% [init_orientation, init_location] = find_initial_pose();
load('init_location.mat')
load('init_orientation.mat')
cam_in_world_orientations(:,:, 1) = init_orientation;
cam_in_world_locations(:,:, 1) = init_location;

% Visualise the pose for the initial frame
edges = [[1, 1, 1, 2, 2, 3, 3, 4, 5, 5, 6, 7]
    [2, 4, 5, 3, 6, 4, 7, 8, 6, 8, 7, 8]];
figure()
hold on;
imshow(char(Filenames(1)), 'InitialMagnification', 'fit');
title(sprintf('Initial Image Camera Pose'));
%   Plot bounding box
points = project3d2image(vertices',camera_params, cam_in_world_orientations(:,:,1), cam_in_world_locations(:, :, 1), "NORMAL");
for j=1:12
    plot(points(1, edges(:, j)), points(2, edges(:,j)), 'color', 'b');
end
hold off;


%% IRLS nonlinear optimisation

% Now you need to implement the method of iteratively reweighted least squares (IRLS)
% to optimise reprojection error between consecutive image frames

% Method steps:
% 1) Project SIFT keypoints from the initial frame (image i) to the object using the
% initial camera pose and the 3D ray intersection code from the task 1. 
% This will give you 3D coordinates (in the world coordinate system) of the
% SIFT keypoints from the initial frame (image i) that correspond to the object
% 2) Find matches between SIFT keypoints from the initial frame (image i) and the
% subsequent frame (image i+1) using vl_ubcmatch() from VLFeat library
% 3) Reproject these matches using corresponding 3D coordinates from the
% step 1 and the initial camera pose back to the subsequent frame (image i+1)
% 4) Compute the reprojection error between pixels from SIFT
% matches for the subsequent frame (image i+1) and from reprojected matches
% from step 3
% 5) Compute Jacobian of the reprojection error with respect to the pose
% parameters and apply IRLS to iteratively update the camera pose for the subsequent frame (image i+1)
% 6) Now the subsequent frame (image i+1) becomes the initial frame for the
% next subsequent frame (image i+2) and the method continues until camera poses for all
% images are estimated

% We suggest you to validate the correctness of the Jacobian implementation
% either using Symbolic toolbox or finite differences approach

% TODO: Implement IRLS method for the reprojection error optimisation
threshold_irls = 0.005; % update threshold for IRLS
lambda = 0.001;
N = 20; % number of iterations
threshold_ubcmatch = 20; % matching threshold for vl_ubcmatch()
vert1 = vertices(faces(:,1),:);
vert2 = vertices(faces(:,2),:);
vert3 = vertices(faces(:,3),:);
num_samples = 5000;
num_images = size(Filenames,2);
u = threshold_irls + 1;
for i=2:num_images
    fprintf("image %d",i);
    rotMatrix = cam_in_world_orientations(:,:,i-1 );
    translation_vector = cam_in_world_locations(:,:,i-1 );
    [backCoords,backDescriptors] = back_projection(num_samples,keypoints{i-1},descriptors{i-1},rotMatrix,translation_vector,vert1,vert2,vert3,camera_params);
    currModel.coords3d = backCoords;
    currModel.descriptors = backDescriptors; %previous model
    matches = vl_ubcmatch(descriptors{i}, currModel.descriptors, threshold_ubcmatch);
    points_3d = currModel.coords3d(matches(2,:),:);
    theta = [rotationMatrixToVector(rotMatrix) translation_vector];
    true_2d = keypoints{i}(1:2,matches(1,:));
    u = threshold_irls + 1;
    for j=1:N
        if u < threshold_irls
            disp("\nearly stop at iter");
            disp(j);
            break
        end
        v = theta(1:3);
        points_uvw = project3d2image(points_3d',camera_params,rotationVectorToMatrix(theta(1:3)),theta(4:end),"uvw");
        points_2d = (points_uvw ./ points_uvw(3,:));
        points_2d = points_2d(1:2,:);
    
        e = compute_distances(points_2d,true_2d); %compute ProjM - m
        sigma= 1.48257968 * mad(e);
        [err,W] = ro_calculate(e,sigma);
        E_init = sum(err);
        
        v_X=skewer(v);
        I_R1 = (eye(3)-rotMatrix) * [1 0 0]';
        I_R2 = (eye(3)-rotMatrix) * [0 1 0]';
        I_R3 = (eye(3)-rotMatrix) * [0 0 1]';
        dR_v1 = (v(1)*v_X + skewer(cross(v,I_R1)))*rotMatrix ;
        dR_v1 = dR_v1/norm(v)^2;
        dR_v2 = (v(2)*v_X + skewer(cross(v,I_R2)))*rotMatrix ;
        dR_v2 = dR_v2/norm(v)^2;
        dR_v3 = (v(3)*v_X + skewer(cross(v,I_R3)))*rotMatrix ;
        dR_v3 = dR_v3/norm(v)^2;
%         [dR_v1,dR_v2,dR_v3] = test(v);
        J = compute_jacobian(points_3d,points_uvw,camera_params.IntrinsicMatrix,dR_v1,dR_v2,dR_v3);
        delta = inv(J'*W*J + lambda*eye(6)) * -1*(J'*W*e);
        temp_theta = theta + delta';
        %%COMPUTING NEW ENERGY
        points_2d_new = project3d2image(points_3d',camera_params,rotationVectorToMatrix(temp_theta(1:3)),temp_theta(4:end),"NORMAL");
%         imshow(char(Filenames(i)));
%         scatter(points_2d_new(1,:),points_2d_new(2,:));
        d_new = compute_distances(points_2d_new,true_2d);

        sigma_new= 1.48257968 * mad(d_new);
        [err_new,W_new] = ro_calculate(d_new,sigma_new);
        E_new = sum(err_new);
        if E_new > E_init
            lambda=10*lambda;
        else
            lamda = lambda/10;
            theta = temp_theta;
            disp(E_init);
            disp(E_new);
            disp("Accepted");
        end
        
        u = norm(delta);
    end
    cam_in_world_orientations(:,:,i) = rotationVectorToMatrix(theta(1:3));
    cam_in_world_locations(:,:,i) = theta(4:end);
    %%CURRENTLY BEING IMPLEMENTED
end


%% Plot camera trajectory in 3D world CS + cameras

figure()
% Predicted trajectory
visualise_trajectory(vertices, edges, cam_in_world_orientations, cam_in_world_locations, 'Color', 'b');
hold on;
% Ground Truth trajectory
visualise_trajectory(vertices, edges, gt_valid.orientations, gt_valid.locations, 'Color', 'g');
hold off;
title('\color{green}Ground Truth trajectory \color{blue}Predicted trajectory')

%% Visualize bounding boxes

figure()
for i=1:num_files
    
    imshow(char(Filenames(i)), 'InitialMagnification', 'fit');
    title(sprintf('Image: %d', i))
    hold on
    % Ground Truth Bounding Boxes
    points_gt = project3d2image(vertices',camera_params, gt_valid.orientations(:,:,i), gt_valid.locations(:, :, i), "NORMAL");
    % Predicted Bounding Boxes
    points_pred = project3d2image(vertices',camera_params, cam_in_world_orientations(:,:,i), cam_in_world_locations(:, :, i), "NORMAL");
    for j=1:12
        plot(points_gt(1, edges(:, j)), points_gt(2, edges(:,j)), 'color', 'g');
        plot(points_pred(1, edges(:, j)), points_pred(2, edges(:,j)), 'color', 'b');
    end
    hold off;
    
    filename = fullfile(results_path, strcat('image', num2str(i), '.png'));
    saveas(gcf, filename)
end

%% Bonus part

% Save estimated camera poses for the validation sequence using TUM Ground-truth trajectories file
% format: https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
% Then estimate Absolute Trajectory Error (ATE) and Relative Pose Error for
% the validation sequence using python tools from: https://vision.in.tum.de/data/datasets/rgbd-dataset/tools

% Save estimated camera poses for the test sequence using TUM Ground-truth
% trajectories file format

% Send us this file with the estimated camera poses for the evaluation
% If the code and results are good you will get a bonus for this exercise
% We are expecting the mean absolute translational error (from ATE) to be
% approximately less than 1cm

% TODO: Estimate ATE and RPE for validation and test sequences

