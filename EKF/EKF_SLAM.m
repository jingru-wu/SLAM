%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  16833 Robot Localization and Mapping  % 
%  Assignment #2                         %
%  EKF-SLAM                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;
clc;

%==== TEST: Setup uncertainity parameters (try different values!) ===
sig_x = 0.25;
sig_y = 0.1;
sig_alpha = 0.1;
sig_beta = 0.01;
sig_r = 0.08;

%=======
% sig_x = 0.25;
% sig_y = 0.1;
% sig_alpha = 0.1;
% sig_beta = 0.1;
% sig_r = 0.8;

%==== Generate sigma^2 from sigma ===
sig_x2 = sig_x^2;
sig_y2 = sig_y^2;
sig_alpha2 = sig_alpha^2;
sig_beta2 = sig_beta^2;
sig_r2 = sig_r^2;

%==== Open data file ====
fid = fopen('../data/data.txt');

%==== Read first measurement data ====
tline = fgets(fid);
arr = str2num(tline);
measure = arr';
t = 1;
 
%==== Setup control and measurement covariances ===
control_cov = diag([sig_x2, sig_y2, sig_alpha2]);
measure_cov = diag([sig_beta2, sig_r2]);

%==== Setup initial pose vector and pose uncertainty ====
pose = [0 ; 0 ; 0];
pose_cov = diag([0.02^2, 0.02^2, 0.1^2]);


%==== TODO: Setup initial landmark vector landmark[] and covariance matrix landmark_cov[] ====
%==== (Hint: use initial pose with uncertainty and first measurement) ====
% Write your code here...

k = length(measure)/2;
landmark = [];
landmark_cov = zeros(k*2, k*2);

for i = 1:k
   beta = measure(2*i-1);
    r = measure(2*i);
    x = pose(1);
    y = pose(2);
    theta = pose(3);
    lx = x + r * cos(theta + beta);
    ly = y + r * sin(theta + beta);
    
    landmark = [landmark;lx;ly];

    H_L_pose = [1, 0,  -r * sin(theta + beta);
                0, 1,   r * cos(theta + beta)];
             
    H_L_measure = [-r * sin(theta + beta), cos(theta + beta);
                    r * cos(theta + beta), sin(theta + beta)];
              
    landmark_cov(2*i-1:2*i, 2*i-1:2*i) = H_L_pose * pose_cov * H_L_pose' + H_L_measure * measure_cov * H_L_measure';
end

%==== Setup state vector x with pose and landmark vector ====
x = [pose ; landmark];

%==== Setup covariance matrix P with pose and landmark covariances ====
P = [pose_cov zeros(3, 2*k) ; zeros(2*k, 3) landmark_cov];

%==== Plot initial state and conariance ====
last_x = x;
drawTrajAndMap(x, last_x, P, 0);

%==== Read control data ====
tline = fgets(fid);
count=0;
while ischar(tline)
    arr = str2num(tline);
    d = arr(1);
    alpha = arr(2);
    
    %==== TODO: Predict Step ====
    %==== (Notice: predict state x_pre[] and covariance P_pre[] using input control data and control_cov[]) ====
    
    % Write your code here...

    xt = last_x(1);
    yt = last_x(2);
    theta = last_x(3);
    x_pre=last_x;
    x_pre(1:3,:) = x_pre(1:3,:) + [d*cos(theta);
                                   d*sin(theta); 
                                    alpha];
                  
    G = [[1, 0, -d*sin(theta);
          0, 1,  d*cos(theta);
          0, 0, 1], zeros(3, 2*k); zeros(2*k, 3), eye(2*k, 2*k)];
      
    F = [cos(theta),    -sin(theta),    0; 
         sin(theta),     cos(theta),    0;
         0         ,         0,         1];
           

    R = [F * control_cov * F', zeros(3, 2*k); zeros(2*k, 2*k+3)];
    P_pre = G * P * G' + R;
    
    %==== Draw predicted state x_pre[] and covariance P_pre[] ====
    drawTrajPre(x_pre, P_pre);
    
    %==== Read measurement data ====
    tline = fgets(fid);
    arr = str2num(tline);
    measure = arr';
    
    %==== TODO: Update Step ====
    %==== (Notice: update state x[] and covariance P[] using input measurement data and measure_cov[]) ====
    
    % Write your code here...

    for i = 1:k
        xt = x_pre(1);
        yt = x_pre(2);
        
        theta = x_pre(3);
        lx = x_pre(2*i+2);
        ly = x_pre(2*i+3);
        
        var = (lx-xt)^2 + (ly-yt)^2;
        % Jaccobian of Z wrt Pt:X,Y,theta
        H_measure_pose = [(ly-yt)/var,        -(lx-xt)/var      ,   -1; 
                          -(lx-xt)/sqrt(var), -(ly-yt)/sqrt(var),   0];

                      
        % Jaccobian of Z wrt landmark Lx,Ly
        H_measure_land = [-(ly-yt)/var     ,       (lx-xt)/var;
                          (lx-xt)/sqrt(var), (ly-yt)/sqrt(var)];
        
        H = [H_measure_pose, zeros(2, (i-1)*2), H_measure_land, zeros(2, (k-i)*2)];
        % Kalman gain
        
        K = P_pre * H' / (H * P_pre * H' + measure_cov);
        
        z = measure(2*i-1:2*i);
        
        h = [wrapToPi(atan2(ly-yt,lx-xt) - theta);  
             sqrt(var)];
         
        x_pre = x_pre + K * (z - h);
        P_pre = (eye(3+2*k) - K * H) * P_pre;
    end
    x = x_pre;
    P = P_pre;
    
    %==== Plot ====   
    drawTrajAndMap(x, last_x, P, t);
    last_x = x;
    
    %==== Iteration & read next control data ===
    t = t + 1;
    tline = fgets(fid);
end


%% ==== EVAL: Plot ground truth landmarks ====



% Write your code here...
landmark_gt = [3, 6, 3, 12, 7, 8, 7, 14, 11, 6, 11, 12];
landmark_actual=reshape(landmark_gt,[2,6])';
scatter ( landmark_actual ( : , 1 ) , landmark_actual ( : , 2 ) , 'rx');

%% Error
for i = 1:k
    pred_loc = x(3+2*i-1:3+2*i);
    gt_loc = landmark_gt(2*i-1:2*i)';
    
    Error_Euclidean(i) = sqrt(sum((pred_loc - gt_loc).^2));
    Error_Mahalanobis(i) = sqrt((pred_loc - gt_loc)' * inv(P(3+2*i-1:3+2*i, 3+2*i-1:3+2*i)) * (pred_loc - gt_loc));
    
end
Error_Euclidean = Error_Euclidean'
Error_Mahalanobis=Error_Mahalanobis'
%==== Close data file ====
fclose(fid);
