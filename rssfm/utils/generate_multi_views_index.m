function [camera_set, p3d,p3d_gt, pram, flag] = generate_multi_views_index(w_level, d_level, measurenet_level, scale, is_randn_degree)
num_camera = 5;
%% Camera setting 
% Intrinsic matrix
% Create camera projection matrices
% Here are the given parameters of the camera:
H = 1080; % height of image in pixels
W = 1280; % width of image in pixels
fx = 1037.52; % focal length in pixels
fy = 1043.32; % focal length in pixels
cx = 640; % optical center
cy = 540;
K = [ fx 0 cx ; % Intrinsic camera parameter matrix
    0 fy cy ;
    0 0 1 ];
t_v = 0.00004625;  % frame scanning speed per row default 30 fps
% t_v = 30 * t_v;
cameraParams = cameraParameters('IntrinsicMatrix',K');
sigma = [measurenet_level 0; 0 measurenet_level];
R_sig = chol(sigma);
%% Generate 3D scene 
p3d = generate3DCube(1.5);
p3d_gt = p3d;
point_num = size(p3d, 1);
%% Simulate camera ego-motion
num_points = point_num;
num_measurement = num_camera * num_points;

% camera one
w = 2 * (rand(3,1) - 0.5 * ones(3, 1));
d = 2 * (rand(3,1) - 0.5 * ones(3, 1));
w_gt = w / norm(w) * (pi * w_level / 180) * t_v;
d_gt = d / norm(d) * d_level * t_v;
camera_pose_1.w_gt = w_gt;
camera_pose_1.d_gt = d_gt;


r = [1 0 0, pi];
R = axang2rotm(r);
t = [0; 0; scale]; 
camera_pose_1.gt_oritation = R;
camera_pose_1.gt_translation = t;
[Rot_Rows, trans_Rows] = linearEgoMotion(R,t,w_gt,d_gt,cameraParams);
[p2d_1,p3d_RS, flag_1] = RSWorld2Image(p3d_gt, Rot_Rows, trans_Rows, cameraParams);
camera_pose_1.gt_feature_point = p2d_1;
noise = randn(num_points, 2) * R_sig;
p2d_1 = p2d_1 + noise;
camera_pose_1.feature_point = p2d_1;
camera_pose_1.oritation = camera_pose_1.gt_oritation;
camera_pose_1.translation = camera_pose_1.gt_translation;


% camera two
w = 2 * (rand(3,1) - 0.5 * ones(3, 1));
d = 2 * (rand(3,1) - 0.5 * ones(3, 1));
w_gt = w / norm(w) * (pi * w_level / 180) * t_v;
d_gt = d / norm(d) * d_level * t_v;
camera_pose_2.w_gt = w_gt;
camera_pose_2.d_gt = d_gt;

if is_randn_degree
    r = [1 1 0 (rand(1) * 0.4) * pi + pi];
else
    r = [1 1 0 0.1 * 2.5 * pi + pi];
end

R = axang2rotm(r);
t = [0; 0; scale]; 
camera_pose_2.gt_oritation = R;
camera_pose_2.gt_translation = t;
[Rot_Rows, trans_Rows] = linearEgoMotion(R,t,w_gt,d_gt,cameraParams);
[p2d_2,p3d_RS, flag_2] = RSWorld2Image(p3d_gt, Rot_Rows, trans_Rows, cameraParams);
camera_pose_2.gt_feature_point = p2d_2;
noise = randn(num_points, 2) * R_sig;
p2d_2 = p2d_2 + noise;
camera_pose_2.feature_point = p2d_2;



% camera three
w = 2 * (rand(3,1) - 0.5 * ones(3, 1));
d = 2 * (rand(3,1) - 0.5 * ones(3, 1));
w_gt = w / norm(w) * (pi * w_level / 180) * t_v;
d_gt = d / norm(d) * d_level * t_v;
camera_pose_3.w_gt = w_gt;
camera_pose_3.d_gt = d_gt;

if is_randn_degree
    r = [1 -1 0 (rand(1) * 0.4 + 0.5) * pi + pi];
else
    r = [1 -1 0 0.1 * 2.5 * pi + pi];
end
R = axang2rotm(r);
t = [0; 0; scale]; 
camera_pose_3.gt_oritation = R;
camera_pose_3.gt_translation = t;
[Rot_Rows, trans_Rows] = linearEgoMotion(R,t,w_gt,d_gt,cameraParams);
[p2d_3,p3d_RS, flag_3] = RSWorld2Image(p3d_gt, Rot_Rows, trans_Rows, cameraParams);
camera_pose_3.gt_feature_point = p2d_3;
noise = randn(num_points, 2) * R_sig;
p2d_3 = p2d_3 + noise;
camera_pose_3.feature_point = p2d_3;

% camera four
w = 2 * (rand(3,1) - 0.5 * ones(3, 1));
d = 2 * (rand(3,1) - 0.5 * ones(3, 1));
w_gt = w / norm(w) * (pi * w_level / 180) * t_v;
d_gt = d / norm(d) * d_level * t_v;
camera_pose_4.w_gt = w_gt;
camera_pose_4.d_gt = d_gt;

if is_randn_degree
    r = [1 1 0 -(rand(1) * 0.4) * pi + pi];
else
    r = [1 1 0 (- 0.1 * 2.5 * pi + pi)];
end
R = axang2rotm(r);
t = [0; 0; scale]; 
camera_pose_4.gt_oritation = R;
camera_pose_4.gt_translation = t;
[Rot_Rows, trans_Rows] = linearEgoMotion(R,t,w_gt,d_gt,cameraParams);
[p2d_4,p3d_RS, flag_4] = RSWorld2Image(p3d_gt, Rot_Rows, trans_Rows, cameraParams);
camera_pose_4.gt_feature_point = p2d_4;
noise = randn(num_points, 2) * R_sig;
p2d_4 = p2d_4 + noise;
camera_pose_4.feature_point = p2d_4;

% camera five
w = 2 * (rand(3,1) - 0.5 * ones(3, 1));
d = 2 * (rand(3,1) - 0.5 * ones(3, 1));
w_gt = w / norm(w) * (pi * w_level / 180) * t_v;
d_gt = d / norm(d) * d_level * t_v;
camera_pose_5.w_gt = w_gt;
camera_pose_5.d_gt = d_gt;

if is_randn_degree
    r = [1 -1 0 -(rand(1) * 0.4 + 0.5) * pi + pi];
else
    r = [1 -1 0 (- 0.1 * 2.5 * pi + pi)];
end
R = axang2rotm(r);
t = [0; 0; scale]; 
camera_pose_5.gt_oritation = R;
camera_pose_5.gt_translation = t;
[Rot_Rows, trans_Rows] = linearEgoMotion(R,t,w_gt,d_gt,cameraParams);
[p2d_5,p3d_RS, flag_5] = RSWorld2Image(p3d_gt, Rot_Rows, trans_Rows, cameraParams);
camera_pose_5.gt_feature_point = p2d_5;
noise = randn(num_points, 2) * R_sig;
p2d_5 = p2d_5 + noise;
camera_pose_5.feature_point = p2d_5;
flag = flag_5 | flag_4 | flag_3 | flag_2 | flag_1;




% ----------------------------- matlab official relative pose and triangulate -----------------------------
E = estimateEssentialMatrix(p2d_1 ,p2d_2, cameraParams);
[relativeOrientation_1,relativeLocation_1] = relativeCameraPose(E,cameraParams,p2d_1,p2d_2);
E = estimateEssentialMatrix(p2d_1 ,p2d_3, cameraParams);
[relativeOrientation_2,relativeLocation_2] = relativeCameraPose(E,cameraParams,p2d_1,p2d_3);
E = estimateEssentialMatrix(p2d_1 ,p2d_4, cameraParams);
[relativeOrientation_3,relativeLocation_3] = relativeCameraPose(E,cameraParams,p2d_1,p2d_4);
E = estimateEssentialMatrix(p2d_1 ,p2d_5, cameraParams);
[relativeOrientation_4,relativeLocation_4] = relativeCameraPose(E,cameraParams,p2d_1,p2d_5);

scale_1 = norm(camera_pose_1.gt_oritation' * camera_pose_1.gt_translation - camera_pose_2.gt_oritation' * camera_pose_2.gt_translation);
scale_2 = norm(camera_pose_1.gt_oritation' * camera_pose_1.gt_translation - camera_pose_3.gt_oritation' * camera_pose_3.gt_translation);
scale_3 = norm(camera_pose_1.gt_oritation' * camera_pose_1.gt_translation - camera_pose_4.gt_oritation' * camera_pose_4.gt_translation);
scale_4 = norm(camera_pose_1.gt_oritation' * camera_pose_1.gt_translation - camera_pose_5.gt_oritation' * camera_pose_5.gt_translation);
camera_pose_2.oritation = relativeOrientation_1 * camera_pose_1.gt_oritation;
camera_pose_2.translation = ((camera_pose_1.translation' - scale_1 * relativeLocation_1) * relativeOrientation_1')';
camera_pose_3.oritation = relativeOrientation_2 * camera_pose_1.gt_oritation;
camera_pose_3.translation = ((camera_pose_1.translation' - scale_2 * relativeLocation_2) * relativeOrientation_2')';
camera_pose_4.oritation = relativeOrientation_3 * camera_pose_1.oritation;
camera_pose_4.translation = ((camera_pose_1.translation' - scale_3 * relativeLocation_3) * relativeOrientation_3')';
camera_pose_5.oritation = relativeOrientation_4 * camera_pose_1.oritation;
camera_pose_5.translation = ((camera_pose_1.translation' - scale_4 * relativeLocation_4) * relativeOrientation_4')';

tracks = [];
for i = 1:num_points
    feature_uv = [p2d_1(i, :);p2d_2(i, :);p2d_3(i, :);p2d_4(i, :);p2d_5(i, :)];
    track_d = pointTrack([1,2,3,4,5], feature_uv);
    tracks = [tracks, track_d];
end
ViewId = uint32([1;2;3;4;5]);
absolute_pose_1 = rigid3d(camera_pose_1.oritation, - camera_pose_1.translation' * camera_pose_1.oritation);
absolute_pose_2 = rigid3d(camera_pose_2.oritation, - camera_pose_2.translation' * camera_pose_2.oritation);
absolute_pose_3 = rigid3d(camera_pose_3.oritation, - camera_pose_3.translation' * camera_pose_3.oritation);
absolute_pose_4 = rigid3d(camera_pose_4.oritation, - camera_pose_4.translation' * camera_pose_4.oritation);
absolute_pose_5 = rigid3d(camera_pose_5.oritation, - camera_pose_5.translation' * camera_pose_5.oritation);
AbsolutePose = [absolute_pose_1;absolute_pose_2;absolute_pose_3;absolute_pose_4;absolute_pose_5];
camPoses = table(ViewId, AbsolutePose);

focalLength = [fx, fy];
principalPoint = [cx, cy];
imageSize = [H, W];
intrinsics_matrix = cameraIntrinsics(focalLength, principalPoint, imageSize);
xyzPoints = triangulateMultiview(tracks, camPoses, intrinsics_matrix);


%----------------------------- set other config -----------------------------
p3d = xyzPoints;
pose_set(1) = camera_pose_1; 
pose_set(2) = camera_pose_2;
pose_set(3) = camera_pose_3;
pose_set(4) = camera_pose_4;
pose_set(5) = camera_pose_5;
camera_set = pose_set;



pram.K_matrix = K;
pram.num_measurement = num_measurement;
pram.num_points = num_points;
pram.num_points_all = point_num;
pram.num_camera = num_camera;
pram.H = H; 
pram.W = W; 
pram.fx = fx;
pram.fy = fy;
pram.cx = cx; 
pram.cy = cy;

end