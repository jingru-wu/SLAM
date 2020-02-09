function [tform valid_pair_num error] = getRigidTransform(new_pointcloud, ref_pointcloud, ref_normals)
    
    %==== Initialize parameters ====
    iter_num = 6;
    d_th = 0.05;
    m = size(new_pointcloud.Location, 1);
    n = size(new_pointcloud.Location, 2);
    tform = affine3d(eye(4));
    
    %==== Main iteration loop ====
    for iter = 1:iter_num
        
        %==== Set variables ====
        new_pts = new_pointcloud.Location; % 30*40*3
        ref_pts = ref_pointcloud.Location;
        
        %==== For each reference point, find the closest new point within a local patch of size 3-by-3 ====        
        %==== (Notice: assoc_pts[] has the same size and format as new_pts[] and ref_pts[]) ====
        %==== (Notice: assoc_pts[i, j, :] = [0 0 0] iff no point in new_pts[] matches to ref_pts[i, j, :]) ====
        assoc_pts = findLocalClosest(new_pts, ref_pts, m, n, d_th);
        
        %==== Set the sizes of matrix A[] and vertor b[] of normal equation: A'*A*x = A'*b ====
        A = zeros(m*n, 6);
        b = zeros(m*n, 1);
        
        %==== declare the number of point pairs that are used in this iteration ==== 
        valid_pair_num = 0;
        
        %==== TODO: Assign values to A[] and b[] ====
        %==== (Notice: the format of the desired 6-vector is: xi = [beta gamma alpha t_x t_y t_z]') ====

        % Write your code here...
        
        %==== TODO: Solve for the 6-vector xi[] of rigid body transformation ====
%         tic;
        for i = 1:m
            for j = 1:n
                if all(assoc_pts(i, j, :) == 0)
                    % no point matched, invalid
                    continue
                end
                valid_pair_num = valid_pair_num + 1;
                
                Vetex=assoc_pts(i, j, :);
                G = [toSkewSym(Vetex), eye(3)];
                
%                 Normal = ;
                Normal = reshape(ref_normals(i, j, :), [3, 1]);
                
%                 V_ref=;
%                 V_assoc=;
                b(i*n+j-n, :) = Normal' * (reshape(ref_pts(i, j, :),[3,1])-reshape(assoc_pts(i, j, :),[3,1]));
                A(i*n+j-n, :) = (G' * Normal)';
            end
        end
        
        
        
        % Write your code here...
        A_sparse = sparse(A);
        [xi, ~] = solve_chol(A_sparse, b);
%         xi = xi';


        %==== Coerce xi[] back into SE(3) ====
        %==== (Notice: tmp_tform[] is defined in the format of right-multiplication) ====
        R = toSkewSym(xi(1:3)) + eye(3);
        [U,~,V] = svd(R);
        R = U*V';
        T = [R [0 ; 0 ; 0] ; [xi(4:6)' 1]];
        tmp_tform = affine3d(T);
        
        %==== Updates the transformation and the pointcloud for the next iteration ====
        %==== (uses affine3d() and pctransform() functions) ====
        %==== (note the format of tform[] and the affine3d() function) ====
        tform = affine3d(tmp_tform.T*tform.T);
        if iter ~= iter_num
            new_pointcloud = pctransform(new_pointcloud, tmp_tform);
        end
        
    end
    
    %==== Find RMS error of point-plane registration ====
    error = sqrt(sum((A*xi - b).^2)/valid_pair_num);
end
        
