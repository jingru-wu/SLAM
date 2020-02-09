% CREATE_AB_LINEAR
% 16-833 Spring 2019 - *Stub* Provided
% Computes the A and b matrices for the 2D linear SLAM problem
%
% Arguments: 
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
%                 obs(:,3) - x-value of landmark measurement
%                 obs(:,4) - y-value of landmark measurement
%     sigma_o - Covariance matrix corresponding to the odometry
%               measurements
%     sigma_l - Covariance matrix corresponding to the landmark
%               measurements
% Returns:
%     A       - MxN matrix
%     b       - Mx1 vector
%
function [As, b] = create_Ab_linear(odom, obs, sigma_o, sigma_l)


% Useful Constants
n_poses = size(odom, 1) + 1; % +1 for prior on the first pose
n_landmarks = max(obs(:,2));

n_odom = size(odom, 1);
n_obs  = size(obs, 1);

% Dimensions of state variables and measurements (all 2 in this case)
p_dim = 2;
l_dim = 2;
o_dim = size(odom, 2);
m_dim = size(obs(1, 3:end), 2);

% A matrix is MxN, b is Mx1
N = p_dim*n_poses + l_dim*n_landmarks;
M = o_dim*(n_odom+1) + m_dim*n_obs;     % +1 for prior on the first pose

% Initialize matrices
A = zeros(M, N);
b = zeros(M, 1);

% Add odometry and landmark measurements to A, b - including prior on first
% pose

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%jaccobian of measurement wrt
     %Rx_t-1,  Ry_t-1 ,  Rx_t,    Ry_t
Ho = [-1,        0,       1,      0;  %delta x
      0,        -1,       0,      1]; %delta y
  
     %Rx_t, Ry_t ,  lx,  ly
Hm = [-1,     0,     1,   0;   %dx
      0,     -1,     0,   1];  %dy
  
sigma_o = sqrtm(sigma_o);
sigma_l = sqrtm(sigma_l);

A(1, 1)=1;
A(2, 2)=1;
b(1:2)=0;
for i =1:(n_obs+n_odom) %i=1 for prepose
    if i<=(n_odom)
        A_O=inv(sigma_o)*Ho;
        A(2*i+1: 2*i+2 , 2*i-1:2*i+2)=A_O;
        b(2*i+1: 2*i+2 ) = inv(sigma_o) * odom(i, :)';
    end
    if i>(n_odom)  % start from 

        obs_i=obs(i-n_odom,:);
        pose_idx = obs_i(1);
        land_idx = obs_i(2);
        land_mea=obs_i(3:4);
        
        A_M = inv(sigma_l) * Hm;
        A_M_D=A_M(:,1:2);
        A_M_Land=A_M(:,3:4);
        A(2*i+1: 2*i+2, 2*pose_idx-1:2*pose_idx)=A_M_D;
        A(2*i+1: 2*i+2, 2*(n_odom+1)+2*land_idx-1:2*(n_odom+1)+2*land_idx)=A_M_Land;
        b(2*i+1: 2*i+2 ) = inv(sigma_l) * land_mea';
    end
end
%% Make A a sparse matrix 
As = sparse(A);