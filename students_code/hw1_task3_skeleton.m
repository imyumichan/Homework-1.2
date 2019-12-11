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
threshold_irls = 1.8e-5; % update threshold for IRLS
N = 40; % number of iterations
threshold_ubcmatch = 40; % matching threshold for vl_ubcmatch()
vert1 = vertices(faces(:,1),:);
vert2 = vertices(faces(:,2),:);
vert3 = vertices(faces(:,3),:);
num_samples = 5000;
num_images = size(Filenames,2);

% Choose the Jacobian calculation method
jacobian_method = "chainrule";
% jacobian_method = "symbolicmath";

if jacobian_method == "symbolicmath"
    syms r1 r2 r3 t1 t2 t3 u v X Y Z;
    A = intrinsicMatrix';
    r = [r1 r2 r3];
    norm_r = norm(r);
    m = [u v]';
    mat = [0 -r3 -r2;r3 0 -r1;-r2 r1 0];
    R = (eye(3) +(sin(norm_r)*mat)/norm_r  + ((1 - cos(norm_r))*mat*mat)/norm_r^2)';
    M = [X Y Z];
    T = [t1 t2 t3];
    p = [r1 r2 r3 t1 t2 t3];
    M_cam = R*M' + T';
    m_uvw = A*M_cam;
    m_img = m_uvw./m_uvw(3);
    m_img = m_img(1:2);
    res = m_img - m;

    J_ml = jacobian(res, [r1 r2 r3 t1 t2 t3]);
end

for i=2:num_images
    
    fprintf("image %d\n",i);
    rotMatrix = cam_in_world_orientations(:,:,i-1 );
    translation_vector = cam_in_world_locations(:,:,i-1 );
    [backCoords,backDescriptors] = back_projection(num_samples,keypoints{i-1},descriptors{i-1},rotMatrix,translation_vector,vert1,vert2,vert3,camera_params);
    matches = vl_ubcmatch(descriptors{i}, backDescriptors, threshold_ubcmatch);
    points_3d = backCoords(matches(2,:),:);
    true_2d = keypoints{i}(1:2,matches(1,:));
    
    theta = [rotationMatrixToVector(rotMatrix) translation_vector];
    lambda = 0.001;
    u = threshold_irls + 1;
    stop_count = 0;
        
    for j=1:N
        
        if u < threshold_irls
            fprintf("Early stop at %d \n",j);
            break
%             stop_count=stop_count + 1;
%             if stop_count > 2
%                 fprintf("Early stop at %d \n",j);
%                 break
%             end
%         else
%             stop_count = 0;
        end
        
        points_uvw = project3d2image(points_3d',camera_params,rotationVectorToMatrix(theta(1:3)),theta(4:end),"uvw");
        points_2d = points_uvw ./ points_uvw(3,:);
        points_2d = points_2d(1:2,:);

        e = compute_distances(points_2d, true_2d); %compute ProjM - m
        err = ro_calculate(e);
        E_init = sum(err);
        W = weight_mat_create(e);

%         rv = rotationVectorToMatrix(theta(1:3));
%         tv = theta(4:end);
%         J = zeros(size(e,1),6);
%         for q=1:2:size(e,1)
%             
%             J(q:q+1,:) = test(camera_params.IntrinsicMatrix,rv,tv,true_2d(:,uint8(q/2)),points_3d(uint8(q/2),:));
%             
%         end
        
        if jacobian_method == "chainrule"
            % Jacobian Method 1
            % Using Chain Rule
            v = theta(1:3);
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
%             [dR_v1,dR_v2,dR_v3] = test(v);
            J = compute_jacobian(points_3d,points_uvw,camera_params.IntrinsicMatrix',dR_v1,dR_v2,dR_v3);
        else
            % Jacobian Method 2
            % Using Symbolic Math toolbox
            J = zeros(size(e,1), 6);
            for p=1:size(true_2d,2)
                temp_J = subs(J_ml,r,theta(1:3));
                temp_J = subs(temp_J,T,theta(4:6));
                temp_J = subs(temp_J,M,points_3d(p,:));
                temp_J = subs(temp_J,m,true_2d(:,p));
                temp_J = double(temp_J);
                J(2*p-1:2*p,:) = temp_J;
            end
        end
        
        delta = -1 * inv(J'*W*J + lambda*eye(6)) * (J'*W*e);
        temp_theta = theta + delta'; 

        %%COMPUTING NEW ENERGY
        points_2d_new = project3d2image(points_3d',camera_params,rotationVectorToMatrix(temp_theta(1:3)),temp_theta(4:end),"NORMAL");
        %         imshow(char(Filenames(i)));
        %         scatter(points_2d_new(1,:),points_2d_new(2,:));
        e_new = compute_distances(points_2d_new,true_2d);
        err_new = ro_calculate(e_new);
        E_new = sum(err_new);
        if E_new >= E_init
            lambda=10*lambda;
        else
            fprintf("%f, %f => Accepted\n", E_init, E_new);
            disp(theta);
            disp(delta');
            lambda = lambda/10;
            theta = temp_theta;
        end
        u = norm(delta);
    end
    cam_in_world_orientations(:,:,i) = rotationVectorToMatrix(theta(1:3));
    cam_in_world_locations(:,:,i) = theta(4:end);
    fprintf('final theta: '); disp(theta);
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
% for i=1:num_files
%     fprintf('%d %f %f %f %f %f %f %f\n', i, cam_in_world_locations(:,1,i), cam_in_world_locations(:,2,i), ...,
%     cam_in_world_locations(:,3,i));
% end

