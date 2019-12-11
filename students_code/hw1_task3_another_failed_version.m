clear
clc
close all
addpath('helper_functions')

%% Setup
% path to the images folder
path_img_dir = '../data/tracking/valid/img';
% path to object ply file
object_path = '../data/teabox.ply';
% path to results folder
results_path = '../data/tracking/valid/results';

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
intrinsicMatrix = [2960.37845      0        0;
                       0       2960.37845   0;
                   1841.68855  1235.23369   1];

camera_params = cameraParameters('IntrinsicMatrix',intrinsicMatrix);

cam_intrinsics=camera_params;

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
%      img = single(rgb2gray(imread(char(Filenames(i)))));
% 
%     [keypoints{i}, descriptors{i}] = vl_sift(img) ;
% end

% Save sift features and descriptors and load them when you rerun the code to save time
% save('sift_descriptors.mat', 'descriptors')
% save('sift_keypoints.mat', 'keypoints')

load('sift_descriptors.mat');
load('sift_keypoints.mat');

%% Initialization: Compute camera pose for the first image

% As the initialization step for tracking
% you need to compute the camera pose for the first image 
% The first image and it's camera pose will be your initial frame 
% and initial camera pose for the tracking process

% You can use estimateWorldCameraPose() function or your own implementation
% of the PnP+RANSAC from previous tasks

% You can get correspondences for PnP+RANSAC either using your SIFT model from the previous tasks
% or by manually annotating corners (e.g. with mark_images() function)


% TODO: Estimate camera position for the first image
%load('init_location.mat')
cam_in_world_orientations(:,:, 1) = [
    0.8340   -0.5504   -0.0389
   -0.3918   -0.5411   -0.7441
    0.3885    0.6358   -0.6669
];

cam_in_world_locations(:,:, 1)=[-0.2601   -0.5856    0.5161];



%cam_in_world_locations(:,:, 1) = init_orientation;
% load('init_location.mat')
% load('init_orientation.mat')
% cam_in_world_orientations(:,:, 1) = init_orientation;
% cam_in_world_locations(:,:, 1) = init_location;

% Visualise the pose for the initial frame
edges = [[1, 1, 1, 2, 2, 3, 3, 4, 5, 5, 6, 7]
    [2, 4, 5, 3, 6, 4, 7, 8, 6, 8, 7, 8]];
figure()
hold on;
imshow(char(Filenames(1)), 'InitialMagnification', 'fit');
title(sprintf('Initial Image Camera Pose'));
%   Plot bounding box
points = project3d2image(vertices',camera_params, cam_in_world_orientations(:,:,1), cam_in_world_locations(:, :, 1));
for j=1:12
    plot(points(1, edges(:, j)), points(2, edges(:,j)), 'color', 'b');
end
hold off;


%% IRLS nonlinear optimisation

% Now you need to implement the method of iteratively reweighted least squares (IRLS)
% to optimise reprojection error between consecutive image frames

% Method steps:
% 1) Back-project SIFT keypoints from the initial frame (image i) to the object using the
% initial camera pose and the 3D ray intersection code from the task 1. 
% This will give you 3D coordinates (in the world coordinate system) of the
% SIFT keypoints from the initial frame (image i) that correspond to the object
% 2) Find matches between descriptors of back-projected SIFT keypoints from the initial frame (image i) and the
% SIFT keypoints from the subsequent frame (image i+1) using vl_ubcmatch() from VLFeat library
% 3) Project back-projected SIFT keypoints onto the subsequent frame (image i+1) using 3D coordinates from the
% step 1 and the initial camera pose 
% 4) Compute the reprojection error between 2D points of SIFT
% matches for the subsequent frame (image i+1) and 2D points of projected matches
% from step 3
% 5) Implement IRLS: for each IRLS iteration compute Jacobian of the reprojection error with respect to the pose
% parameters and update the camera pose for the subsequent frame (image i+1)
% 6) Now the subsequent frame (image i+1) becomes the initial frame for the
% next subsequent frame (image i+2) and the method continues until camera poses for all
% images are estimated

% We suggest you to validate the correctness of the Jacobian implementation
% either using Symbolic toolbox or finite differences approach

% TODO: Implement IRLS method for the reprojection error optimisation
% You can start with these parameters to debug your solution 
% but you should also experiment with their different values
threshold_irls = 0.0005; % update threshold for IRLS
N = 20; % number of iterations
threshold_ubcmatch = 6; % matching threshold for vl_ubcmatch()



for i=2:size(Filenames,2)
    disp(i);
    disp('im');
    lamda=0.1;
    u=threshold_irls+1;
    P = camera_params.IntrinsicMatrix.'*[cam_in_world_orientations(:,:,i-1) -cam_in_world_orientations(:,:,i-1)*cam_in_world_locations(:,:,i-1).'];
    Q = P(:,1:3);
    q = P(:,4);
    orig = -inv(Q)*q; % this corresponds to C
%    Section to be deleted ends here
    inter_coord3d=[];
    inter_descriptors=[];
    for j=1:size(descriptors{i-1},2)
        
    % TODO: Perform intersection between a ray and the object
    % You can use TriangleRayIntersection to find intersections
    % Pay attention at the visible faces from the given camera position
        keypoints_coordinates=keypoints{i-1}(1:2,:);
        dir=keypoints_coordinates(:,j);
        dir=[dir;1];
        dir=Q\dir;

        vert1 = vertices(faces(:,1),:);
        vert2 = vertices(faces(:,2),:);
        vert3 = vertices(faces(:,3),:);

        [intersect, ~, ~, ~, xcoor] = TriangleRayIntersection(orig, dir, vert1, vert2, vert3, 'planeType', 'one sided');
        xcoor(any(isnan(xcoor)'),:) = [];
        if ~isempty(xcoor)
            inter_coord3d=[inter_coord3d;xcoor];
            inter_descriptors = [inter_descriptors,descriptors{i-1}(:,j)];
        end
    end  
%     if isempty(inter_coord3d)
% %         the=norm(p(1:3,:));
% %   
% %         ome = [0, -p(3), p(2); p(1), 0, -p(1); -p(2), p(1), 0];
% %         R_cam = eye(3) + sin(the)/the * ome + (1-cos(the))/(the^2) * ome ^2;
%         cam_in_world_orientations(:,:,i-1) = cam_in_world_orientations(:,:,i-1)';
%         cam_in_world_locations(:,:,i) = -cam_in_world_orientations(:,:,i)*p_new(4:6,:);
%         P = camera_params.IntrinsicMatrix.'*[cam_in_world_orientations(:,:,i-1)' -cam_in_world_orientations(:,:,i-1)'*cam_in_world_locations(:,:,i-1).'];
%         Q = P(:,1:3);
%         q = P(:,4);
%         orig = -inv(Q)*q; % this corresponds to C
% %    Section to be deleted ends here
%         inter_coord3d=[];
%         inter_descriptors=[];
%     for j=1:size(descriptors{i-1},2)
%         
%     % TODO: Perform intersection between a ray and the object
%     % You can use TriangleRayIntersection to find intersections
%     % Pay attention at the visible faces from the given camera position
%         keypoints_coordinates=keypoints{i-1}(1:2,:);
%         dir=keypoints_coordinates(:,j);
%         dir=[dir;1];
%         dir=Q\dir;
% 
%         vert1 = vertices(faces(:,1),:);
%         vert2 = vertices(faces(:,2),:);
%         vert3 = vertices(faces(:,3),:);
% 
%         [intersect, ~, ~, ~, xcoor] = TriangleRayIntersection(orig, dir, vert1, vert2, vert3, 'planeType', 'one sided');
%         xcoor(any(isnan(xcoor)'),:) = [];
%         if ~isempty(xcoor)
%             inter_coord3d=[inter_coord3d;xcoor];
%             inter_descriptors = [inter_descriptors,descriptors{i-1}(:,j)];
%         end
%     end  
%         
%     end
    matches=vl_ubcmatch(descriptors{i}, inter_descriptors, threshold_ubcmatch);
    
    %points_thd 3dpoints(i-1)
    %points_td  projected 3dpoints(i-1)
    %points_otd original2dpoints(i)
    points_thd=inter_coord3d(matches(2,:),:)';
    points_td = project3d2image(points_thd, camera_params, cam_in_world_orientations(:,:,i-1), cam_in_world_locations(:,:,i-1));
    points_otd=keypoints{i}(1:2,matches(1,:));
    
    
    %energy function
    temp_dist=points_otd-points_td;  
    du=sqrt(temp_dist(1,:).^2);
    dv=sqrt(temp_dist(2,:).^2);
    
    e=[du,dv]';
   % sigma=1.48257968*mad(e);
  
    
    %tukey
    c=4.685;
    %e_J=e;
    e=e/mad(e)*0.6745;
    e_e=e;
    for k=1:size(e,1) 
        if(e(k)<=c)
            e(k)=c^2/6.0*((1-(1-(e(k)/c)^2))^3);
        elseif e(k)>c
            e(k)=c^2/6.0;
        end
    end
    
    %w
    %sigma=1.48257968*mad(e);
    %e_w=e/sigma;
    E=sum(e);
    [Rv, Tt] = cameraPoseToExtrinsics(cam_in_world_orientations(:,:,i-1), cam_in_world_locations(:,:,i-1));
    v=rotationMatrixToVector(Rv)';
    t=Tt';
    p=[v;t];
    
    l=0;
    while l<N&&u>threshold_irls
        
        sigma=1.48257968*mad(e_e);
        e_w=e_e/sigma;
        w=[];
        for k=1:size(e_w,1) 
            
            if(e_w(k)<c)
                temp=(1-(e_w(k)/c)^2)^2;
            elseif e_w(k)>=c
                temp=0;
            end
            w=[temp,w];
        end
        W=diag(w);
        
        
        v=p(1:3,:);
        t=p(4:6,:);
        
        
        syms r1 r2 r3 t1 t2 t3 M1 M2 M3
        theta=sqrt(r1*r1+r2*r2+r3*r3);
        M=[M1;M2;M3];
        omega = [0, -r3, r2; r3, 0, -r1; -r2, r1, 0];
        R = eye(3) + sin(theta)/theta * omega + (1-cos(theta))/(theta^2) * omega ^2;
        R=R';
        T=[t1;t2;t3];

        tdpoint_h = intrinsicMatrix'*(R*M+T);
        tdpoint = [tdpoint_h(1) / tdpoint_h(3); tdpoint_h(2) / tdpoint_h(3)];
        temp_jaco=jacobian(tdpoint, [r1; r2; r3; t1; t2; t3]);
        jacobianMatrix=[];
        for k=1:size(points_thd,2)
        
        %tmp=[subs(temp_jaco,{r1, r2, r3, t1, t2, t3, M1, M2, M3},{v(1),v(2),v(3),t(1),t(2),t(3),points_thd(1,k),points_thd(2,k),points_thd(3,k)});tmp];
            r1=v(1);
            r2=v(2);
            r3=v(3);
            t1=t(1);
            t2=t(2);
            t3=t(3);
            M1=points_thd(1,k);
            M2=points_thd(2,k);
            M3=points_thd(3,k);
        
            jacobianMatrix=[double(subs(temp_jaco));jacobianMatrix];
        end
        
        
        
        delta=-((jacobianMatrix'*W*jacobianMatrix+lamda*eye(6))^(-1))*(jacobianMatrix'*W*e);
        p_new=p+delta;
        cam_in_world_orientations(:,:,i) = rotationVectorToMatrix(p_new(1:3,:)');
        
        cam_in_world_locations(:,:,i) = -(cam_in_world_orientations(:,:,i)*p_new(4:6,:))';
        points_td = project3d2image(points_thd, camera_params, cam_in_world_orientations(:,:,i), cam_in_world_locations(:,:,i));
    
        temp_dist=points_otd-points_td;  
        du=sqrt(temp_dist(1,:).^2);
        dv=sqrt(temp_dist(2,:).^2);
        e=[du,dv]'; 
        %e_J=e;
        e=e/mad(e)*0.6745;
        e_e=e;
        for k=1:size(e,1) 
            if(e(k)<=c)
                e(k)=c^2/6.0*((1-(1-(e(k)/c)^2))^3);
            elseif e(k)>c
                e(k)=c^2/6.0;
            end
        end   
        
        E_new=sum(e);
        
        
        if E_new>E
            lamda=10*lamda;
        elseif E_new<=E
            lamda=lamda/10;
            p=p_new;
            E=E_new;
        end
        
        
        u=norm(delta);
        
        
        
%         disp('jacobianMatr');
%         disp(jacobianMatrix);
%         disp('W');
%         disp(W);
        disp('lamda');
        disp(lamda);
%         disp('e');
%         disp(e);

        
        
        disp('delta');
        disp(delta);
        l=l+1;
        disp('l');
        disp(l);
        disp('E_new');
        disp(E_new);
        disp('E');
        disp(E);
        disp('p');
        disp(p);
        disp('u');
        disp(u);
    end
%         the=norm(p(1:3,:));
%   
%         ome = [0, -p(3), p(2); p(3), 0, -p(1); -p(2), p(1), 0];
%         R_cam = eye(3) + sin(the)/the * ome + (1-cos(the))/(the^2) * ome ^2;
%         cam_in_world_orientations(:,:,i) = R_cam;        
        cam_in_world_orientations(:,:,i) = rotationVectorToMatrix(p(1:3,:)')';
        %cam_in_world_locations(:,:,i)=p_new(4:6,:)';
        cam_in_world_locations(:,:,i) = -cam_in_world_orientations(:,:,i)*p_new(4:6,:);
        disp(cam_in_world_orientations(:,:,i));
%     w=[];
%     for k=1:size(e_w,1) 
%         if(e_w(k)<c)
%             temp=(1-(1-(e_w(k)/c)^2))^2;
%         elseif e_w(k)>=c
%             temp=0;
%         end
%         w=[temp,w];
%     end
% %     
%     W=diag(w);
%     
%     %jacobian
%     v=p(1:3,:);
%     t=p(4:6,:);
%     
%     
%     
% 
%  
%     syms r1 r2 r3 t1 t2 t3 M1 M2 M3
%     theta=sqrt(r1*r1+r2*r2+r3*r3);
%     M=[M1;M2;M3];
%     omega = [0, -r3, r2; r3, 0, -r1; -r2, r1, 0];
%     R = eye(3) + sin(theta)/theta * omega + (1-cos(theta))/(theta^2) * omega ^2;
%     T=[t1;t2;t3];
% 
%     tdpoint_h = intrinsicMatrix*(R*M+T);
%     tdpoint = [tdpoint_h(1) / tdpoint_h(3); tdpoint_h(2) / tdpoint_h(3)];
%     temp_jaco=jacobian(tdpoint, [r1; r2; r3; t1; t2; t3]);
%     jacobianMatrix=[];
%     for k=1:size(points_thd,2)
%         
%         %tmp=[subs(temp_jaco,{r1, r2, r3, t1, t2, t3, M1, M2, M3},{v(1),v(2),v(3),t(1),t(2),t(3),points_thd(1,k),points_thd(2,k),points_thd(3,k)});tmp];
%         r1=v(1);
%         r2=v(2);
%         r3=v(3);
%         t1=t(1);
%         t2=t(2);
%         t3=t(3);
%         M1=points_thd(1,k);
%         M2=points_thd(2,k);
%         M3=points_thd(3,k);
%         
%         jacobianMatrix=[double(subs(temp_jaco));jacobianMatrix];
%         
%         
%     end
%     %delta
%     delta=-((jacobianMatrix'*W*jacobianMatrix+lamda*eye(6))^(-1))*(jacobianMatrix'*W*e);
%     p_new=p+dleta;
%     cam_in_world_orientations(:,:,i) = rotationVectorToMatrix(p_new(1:3,:)');
%     cam_in_world_locations(:,:,i) = p_new(4:6,:)';
%     points_td = project3d2image(points_thd, camera_params, cam_in_world_orientations(:,:,i), cam_in_world_locations(:,:,i));
%     
%     temp_dist=points_otd-points_td;  
%     du=sqrt(temp_dist(1,:).^2);
%     dv=sqrt(temp_dist(2,:).^2);
%     e=[du,dv]';
%     
%     for k=1:size(e,1) 
%         if(e(k)<=c)
%             e(k)=c^2/6.0*((1-(1-(e(k)/c)^2))^3);
%         elseif e(k)>c
%             e(k)=c^2/6.0;
%         end
%     end   
%     
%     
%     E_new=e'*e;
%     if E_new>E
%         lamda=10*lamda;
%     elseif E_new<=E
%         lamda=lamda/10;
%         p=p_new;
%     end
%     u=norm(delta);
%     %%
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
    points_gt = project3d2image(vertices',camera_params, gt_valid.orientations(:,:,i), gt_valid.locations(:, :, i));
    % Predicted Bounding Boxes
    points_pred = project3d2image(vertices',camera_params, cam_in_world_orientations(:,:,i), cam_in_world_locations(:, :, i));
    for j=1:12
        plot(points_gt(1, edges(:, j)), points_gt(2, edges(:,j)), 'color', 'g');
        plot(points_pred(1, edges(:, j)), points_pred(2, edges(:,j)), 'color', 'b');
    end
    hold off;
    
    filename = fullfile(results_path, strcat('image', num2str(i), '.png'));
    saveas(gcf, filename)
end

%% Bonus part

% Save estimated camera poses for the validation sequence using Vision TUM trajectory file
% format: https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
% Then estimate Absolute Trajectory Error (ATE) and Relative Pose Error for
% the validation sequence using python tools from: https://vision.in.tum.de/data/datasets/rgbd-dataset/tools
% In this task you should implement you own function to convert rotation matrix to quaternion

% Save estimated camera poses for the test sequence using Vision TUM 
% trajectory file format

% Attach the file with estimated camera poses for the test sequence to your code submission
% If your code and results are good you will get a bonus for this exercise
% We are expecting the mean absolute translational error (from ATE) to be
% approximately less than 1cm

% TODO: Estimate ATE and RPE for validation and test sequences


