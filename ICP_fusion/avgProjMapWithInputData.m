function updated_map = avgProjMapWithInputData(proj_map, input_data, alpha, h, w, is_use, t)

%     ==== Set variables ====
    input_points = input_data.pointcloud.Location;
    input_colors = input_data.pointcloud.Color;
    input_normals = input_data.normals;
    
    proj_points = proj_map.points;
    proj_colors = proj_map.colors;
    proj_normals = proj_map.normals;
    
    proj_ccounts = proj_map.ccounts;
    proj_times = proj_map.times;

%     ==== TODO: Update all the terms in the projected map using the input data ====
%     ==== (Hint: apply is_use[] as a mask in vectorization) ====

%     Write your code here...
    flag=repmat(is_use,[1,1,3]);
    input_points=reshape(input_points(flag),[],3);
    input_colors=reshape(input_colors(flag),[],3);
    input_normals=reshape(input_normals(flag),[],3);

    proj_points=reshape(proj_points(flag),[],3);
    proj_colors=reshape(proj_colors(flag),[],3);
    proj_normals=reshape(proj_normals(flag),[],3);
    proj_ccounts=reshape(proj_ccounts(is_use),[],1);
    alpha=reshape(alpha(is_use),[],1);
    
    updated_points = reshape(proj_map.points,[],3);
    updated_colors = reshape(proj_map.colors,[],3);
    updated_normals = reshape(proj_map.normals,[],3);
    
    updated_ccounts = reshape(proj_map.ccounts,[],1);
    updated_times = reshape(proj_map.times,[],1);
    
    
    
    
    N=proj_ccounts+alpha;
    updated_points(repmat(is_use, 1, 3)) = (proj_ccounts.* proj_points  +alpha .* input_points) ./N;
    updated_colors(repmat(is_use, 1, 3)) = uint8((proj_ccounts.* proj_colors  +alpha .* double(input_colors)) ./N);
    updated_normals(repmat(is_use, 1, 3))= (proj_ccounts.* proj_normals +alpha .* input_normals)./N;
    updated_ccounts(is_use) = proj_ccounts + alpha;
    updated_times(is_use) = t;
    
    
    updated_points = reshape(updated_points, [h, w, 3]);
    updated_colors = reshape(updated_colors, [h, w, 3]);
    updated_normals = reshape(updated_normals, [h, w, 3]);
    updated_ccounts = reshape(updated_ccounts, [h, w]);
    updated_times = reshape(updated_times, [h, w]);
%     ==== Output the updated projected map in a struct ====
    updated_map = struct('points', updated_points, 'colors', updated_colors, 'normals', updated_normals, 'ccounts', updated_ccounts, 'times', updated_times);
        
end


% 
% 
% function updated_map = avgProjMapWithInputData(proj_map, input_data, alpha, h, w, is_use, t)
% 
%     %==== Set variables ====
%     input_points = input_data.pointcloud.Location;
%     input_colors = input_data.pointcloud.Color;
%     input_normals = input_data.normals;
%     
%     proj_points = proj_map.points;
%     proj_colors = proj_map.colors;
%     proj_normals = proj_map.normals;
%     
%     proj_ccounts = proj_map.ccounts;
%     proj_times = proj_map.times;
% 
%     %==== TODO: Update all the terms in the projected map using the input data ====
%     %==== (Hint: apply is_use[] as a mask in vectorization) ====
% 
%     % Write your code here...
%     updated_times=proj_times;
%     
%     N=proj_ccounts+alpha;
%     updated_points = (proj_ccounts.* proj_points  +alpha .* input_points) ./N;
%     
%     updated_colors = uint8((proj_ccounts.* proj_colors  +alpha .* double(input_colors)) ./N);
%     
%     updated_normals= (proj_ccounts.* proj_normals +alpha .* input_normals)./N;
%     
%     updated_ccounts = proj_ccounts + alpha;
%     updated_ccounts(~is_use) = 0;
%     
%     updated_times(is_use) = t;  
% 
% 
%     %==== Output the updated projected map in a struct ====
%     updated_map = struct('points', updated_points, 'colors', updated_colors, 'normals', updated_normals, 'ccounts', updated_ccounts, 'times', updated_times);
%         
% end