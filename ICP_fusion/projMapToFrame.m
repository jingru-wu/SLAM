function [proj_map, proj_flag] = projMapToFrame(fusion_map, h, w, tform, cam_param)
    
    %==== Set parameters ====
    fx = cam_param(1);
    fy = cam_param(2);
    cx = cam_param(3);
    cy = cam_param(4);

    %==== TODO: Project all terms in the fusion map based on other input parameters ====
    %==== (Hint 1: Only project the points in front of the camera) ====
    %==== (Hint 2: Calculate all the projected indices together and use them in vectorization) ====
    %==== (Hint 3: Discard the indices that exceed the frame boundaries) ====

    % Write your code here...

    %% load original data
    location = fusion_map.pointcloud.Location;
    colors = fusion_map.pointcloud.Color;
    normals = fusion_map.normals;
    ccounts = fusion_map.ccounts;
    times = fusion_map.times;
    
    
    %% rigid transform from previous pose
    point_cloud=fusion_map.pointcloud;
    trans_point=pctransform(point_cloud,tform.invert);
    trans_point_location=trans_point.Location;
    
    % hint1: infront of camera
    valid_trans=trans_point_location(:,3)>0; 
    trans_point_location(~valid_trans,:)=0;
    
    %% projection
    % intrinsic matrix
    K = [fx  0   cx;
         0   fy  cy;
         0   0   1 ];
    
    proj_points_location = trans_point_location*K';  % (3*N) =(3*3)*(N*3)' 
                                                     % (N*3) =(N*3)*(3*3)' 
    % normalize
    proj_points_location = proj_points_location./proj_points_location(:,3);
    % projection out of boundary
    valid_proj=proj_points_location(:, 1) > 0 & proj_points_location(:, 1) < h & proj_points_location(:, 2) > 0 & proj_points_location(:, 2) < w;
    
    proj_flag=valid_proj&valid_trans;
%     vaild_proj_points_location=proj_points_location(proj_flag,:);  %n*3
    vaild_proj_points_location=proj_points_location;  %n*3
    % remove points

    proj_points = zeros(h*w, 3);
    proj_colors = zeros(h*w, 3);
    proj_normals = zeros(h*w, 3);
    proj_ccounts = zeros(h*w, 1);
    proj_times = zeros(h*w, 1);
    
    valid_loc=ceil(vaild_proj_points_location);
    valid_loc=valid_loc(proj_flag,:);
    proj_points ( valid_loc(:, 1) * h + valid_loc(:, 2)+1, :) = location(proj_flag,:);
    proj_colors ( valid_loc(:, 1) * h + valid_loc(:, 2)+1, :) = colors(proj_flag,:);
    proj_normals( valid_loc(:, 1) * h + valid_loc(:, 2)+1, :) = normals(proj_flag,:);
    proj_ccounts( valid_loc(:, 1) * h + valid_loc(:, 2)+1) = ccounts(proj_flag);
    proj_times  ( valid_loc(:, 1) * h + valid_loc(:, 2)+1) = times(proj_flag);

    proj_points = reshape(proj_points, [h, w, 3]);
    proj_colors = reshape(proj_colors, [h, w, 3]);
    proj_normals = reshape(proj_normals, [h, w, 3]);
    proj_ccounts = reshape(proj_ccounts, [h, w]);
    proj_times = reshape(proj_times, [h, w]);

    %==== Output the projected map in a struct ====
    %==== (Notice: proj_points[], proj_colors[], and proj_normals[] are all 3D matrices with size h*w*3) ====
    %==== (Notice: proj_ccounts[] and proj_times[] are both 2D matrices with size h*w) ====
    proj_map = struct('points', proj_points, 'colors', proj_colors, 'normals', proj_normals, 'ccounts', proj_ccounts, 'times', proj_times);
        
end
