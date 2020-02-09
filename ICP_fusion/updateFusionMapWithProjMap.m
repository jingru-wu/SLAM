function fusion_map = updateFusionMapWithProjMap(fusion_map, updated_map, h, w, proj_flag)

    %==== TODO: Merge the updated map with the remaining part of the old fusion map ====

    % Write your code here...
    unchange_flag=~(proj_flag);
    unchange_location = fusion_map.pointcloud.Location(unchange_flag,:);
    unchange_colors = fusion_map.pointcloud.Color(unchange_flag,:);
    unchange_normals = fusion_map.normals(unchange_flag,:);
    unchange_ccounts = fusion_map.ccounts(unchange_flag);
    unchange_times = fusion_map.times(unchange_flag);
    
    new_location = reshape(updated_map.points,[h * w,3]);
    new_colors = reshape(updated_map.colors,[h * w,3]);
    new_normals = reshape(updated_map.normals,[h * w,3]);
    new_ccounts = reshape(updated_map.ccounts,[h * w,1]);
    new_times = reshape(updated_map.times,[h * w,1]);
    
    
    map_points = cat(1, unchange_location, new_location);
    map_colors = cat(1, unchange_colors, new_colors);
    map_normals = cat(1, unchange_normals, new_normals);
    map_ccounts = cat(1, unchange_ccounts, new_ccounts);
    map_times = cat(1, unchange_times, new_times);
    
    
    %==== Output the final point-based fusion map in a struct ====
    map_pointcloud = pointCloud(map_points, 'Color', map_colors);
    fusion_map = struct('pointcloud', map_pointcloud, 'normals', map_normals, 'ccounts', map_ccounts, 'times', map_times);
      
end
   