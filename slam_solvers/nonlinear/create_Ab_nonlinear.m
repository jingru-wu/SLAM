% CREATE_AB_NONLINEAR
% 16-833 Spring 2019 - *Stub* Provided
% Computes the A and b matrices for the 2D nonlinear SLAM problem
%
% Arguments: 
%     x       - Current estimate of the state vector
%     odom    - Matrix that contains the odometry measurements
%               between consecutive poses. Each row corresponds to
%               a measurement. 
%                 odom(:,1) - x-value of odometry measurement
%                 odom(:,2) - y-value of odometry measurement
%     obs     - Matrix that contains the landmark measurements and
%               relevant information. Each row corresponds to a
%               measurement.
%                 obs(:,1) - idx of pose at which measurement was 
%                   made
%                 obs(:,2) - idx of landmark being observed
%                 obs(:,3) - bearing theta of landmark measurement
%                 obs(:,4) - range d of landmark measurement
%     sigma_o - Covariance matrix corresponding to the odometry
%               measurements
%     sigma_l - Covariance matrix corresponding to the landmark
%               measurements
% Returns:
%     A       - MxN matrix
%     b       - Mx1 vector
%
function [As, b] = create_Ab_nonlinear(x, odom, obs, sigma_o, sigma_l)
%% Extract useful constants which you may wish to use
n_poses = size(odom, 1) + 1;                % +1 for prior on the first pose
n_landmarks = max(obs(:,2));

n_odom = size(odom, 1);
n_obs  = size(obs, 1);

% Dimensions of state variables and measurements (all 2 in this case)
p_dim = 2;                                  % pose dimension
l_dim = 2;                                  % landmark dimension
o_dim = size(odom, 2);                      % odometry dimension
m_dim = size(obs(1, 3:end), 2);             % landmark measurement dimension

% A matrix is MxN, b is Mx1
N = p_dim*n_poses + l_dim*n_landmarks;
M = o_dim*(n_odom+1) + m_dim*n_obs;         % +1 for prior on the first pose

%% Initialize matrices
A = zeros(M, N);
b = zeros(M, 1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
     %Rx_t-1,  Ry_t-1 ,  Rx_t,    Ry_t
Ho = [-1,        0,       1,      0;  %delta x
      0,        -1,       0,      1]; %delta y
  
sigma_o = sqrtm(sigma_o);
sigma_l = sqrtm(sigma_l);

A(1, 1) = 1;
A(2, 2) = 1;
b(1:2)=0;

for i = 1:n_odom+n_obs
    if i <= n_odom
        pre_x=x(i*2-1);
        pre_y=x(i*2);
        cur_x=x(i*2+1);
        cur_y=x(i*2+2);
        odom_est=odom(i,:);
        ho = meas_odom(pre_x, pre_y, cur_x,cur_y);
        b(2*i+1:2*i+2) = inv(sigma_o) * (odom_est - ho')';
        A_item = inv(sigma_o) * Ho;
        A(2*i+1:2*i+2, 2*i-1:2*i+2) = A_item;
    end
    if i > n_odom
        offset_y = 2*n_odom+2;
        offset_x = 2 * n_poses;
        ods_i = obs(i-n_odom, :);
        pose_idx = ods_i(1);
        land_idx = ods_i(2);
        land_mea=ods_i(3:4);
        cur_x = x(pose_idx*2-1);
        cur_y =  x(pose_idx*2);
        
        land_est = x(offset_y+land_idx*2-1:offset_y+land_idx*2);
        
        land_mea_est = meas_landmark(cur_x, cur_y, land_est(1), land_est(2));
        
        Hm = meas_landmark_jacobian(cur_x, cur_y, land_est(1), land_est(2));
    
        A_M = inv(sigma_l) * Hm;
        A_M_D=A_M(:,1:2);
        A_M_L=A_M(:,3:4);
        A(2*n_odom+2*i+1:2*n_odom+2+2*i, 2*pose_idx-1:2*pose_idx) = A_M_D;
        A(2*n_odom+2*i+1:2*n_odom+2+2*i, 2 * n_poses+2*(land_idx-1)+1:2 * n_poses+2*land_idx) = A_M_L;


        theta = wrapToPi(land_mea(1)-land_mea_est(1));
        r=land_mea(2)-land_mea_est(2);
        b(2*n_odom+2*i+1:2*n_odom+2*i+2) = inv(sigma_l) * [theta;r];
    end
end

%% Make A a sparse matrix 
As = sparse(A);
