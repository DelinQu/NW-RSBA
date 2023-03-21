function [camera_set, p3d,p3d_gt, pram, flag] = generate_degeneracy_views_pitch_index(w_level, d_level, measurenet_level, pitch, scale, is_rand_degree)

camera_num = 5;
num_points = 36;
num_points_all = 36;
num_camera = camera_num;
num_measurement = num_camera * num_points;
%% Camera setting 
% Intrinsic matrix
% Create camera projection matrices
% Here are the given parameters of the camera:
H = 1080; % height of image in pixels
W = 1280; % width of image in pixels
fx = 1037.57521466470; % focal length in pixels
fy = 1043.31575231792; % focal length in pixels
cx = 640; % optical center
cy = 540;
K = [ fx 0 cx ; % Intrinsic camera parameter matrix
    0 fy cy ;
    0 0 1 ];
t_v = 0.00004625;  % frame scanning speed per row default 30 fps
% t_v = 0.00138;
cameraParams = cameraParameters('IntrinsicMatrix',K');
sigma = [measurenet_level 0; 0 measurenet_level];
R_sig = chol(sigma);
%% Generate 3D scene 
% p3d = generate3DSphere(17, 6);   % plane 3D scene 
% p3d_gt = p3d;
% point_id_set(1).point = p3d_gt;
% point_id_set(2).point = p3d_gt;
% point_id_set(3).point = p3d_gt;
% 
% 
% point_id_set(1).index = 1:size(p3d_gt, 1);
% point_id_set(2).index = 1:size(p3d_gt, 1);
% point_id_set(3).index = 1:size(p3d_gt, 1);




p3d = generate3DCube(1.5);
p3d_gt = p3d;
num_points = size(p3d_gt, 1);
num_points_all = size(p3d_gt, 1);
num_measurement = num_camera * num_points;

%% Simulate camera ego-motion
w = 2 * (rand(3,1) - 0.5 * ones(3, 1));
d = 2 * (rand(3,1) - 0.5 * ones(3, 1));
w_gt = w / norm(w) * (pi * w_level / 180) * t_v;
d_gt = d / norm(d) * d_level * t_v;
camera_pose_1.w_gt = w_gt;
camera_pose_1.d_gt = d_gt;



% camera one
r = [1 0 0 pi];
R = axang2rotm(r);
t = [0;0;scale]; 
camera_pose_1.gt_oritation = R;
camera_pose_1.gt_translation = t;
[Rot_Rows, trans_Rows] = linearEgoMotion(R,t,w_gt,d_gt,cameraParams);
[p2d_1,p3d_RS, flag_1] = RSWorld2Image(p3d_gt, Rot_Rows, trans_Rows, cameraParams);

p2d_1 = p2d_1 + randn(num_points, 2) * R_sig;
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
r = [1 0 0 pi];
R = axang2rotm(r);
r = [0 0 1 pitch * pi];
R = axang2rotm(r) * R;

if is_rand_degree
    r = [0 1 0 (rand(1) * 0.5 + 0.5) * pi];
else
    r = [0 1 0 0.3 * pi];
end
R = axang2rotm(r) * R;
t = [0;0;scale]; 
camera_pose_2.gt_oritation = R;
camera_pose_2.gt_translation = t;
[Rot_Rows, trans_Rows] = linearEgoMotion(R,t,w_gt,d_gt,cameraParams);
[p2d_2,p3d_RS, flag_2] = RSWorld2Image(p3d_gt, Rot_Rows, trans_Rows, cameraParams);

p2d_2 = p2d_2 + randn(num_points, 2) * R_sig;
camera_pose_2.feature_point = p2d_2;



% camera three
w = 2 * (rand(3,1) - 0.5 * ones(3, 1));
d = 2 * (rand(3,1) - 0.5 * ones(3, 1));
w_gt = w / norm(w) * (pi * w_level / 180) * t_v;
d_gt = d / norm(d) * d_level * t_v;
camera_pose_3.w_gt = w_gt;
camera_pose_3.d_gt = d_gt;
r = [1 0 0 pi];
R = axang2rotm(r);
r = [0 0 1 -pitch * pi];
R = axang2rotm(r) * R;

if is_rand_degree
    r = [0 1 0 (rand(1) * 0.5) * pi];
else
    r = [0 1 0 0.7 * pi];
end
R = axang2rotm(r) * R;
t = [0;0;scale]; 
camera_pose_3.gt_oritation = R;
camera_pose_3.gt_translation = t;
[Rot_Rows, trans_Rows] = linearEgoMotion(R,t,w_gt,d_gt,cameraParams);
[p2d_3,p3d_RS, flag_3] = RSWorld2Image(p3d_gt, Rot_Rows, trans_Rows, cameraParams);
p2d_3 = p2d_3 + randn(num_points, 2) * R_sig;
camera_pose_3.feature_point = p2d_3;

% camera four
w = 2 * (rand(3,1) - 0.5 * ones(3, 1));
d = 2 * (rand(3,1) - 0.5 * ones(3, 1));
w_gt = w / norm(w) * (pi * w_level / 180) * t_v;
d_gt = d / norm(d) * d_level * t_v;
camera_pose_4.w_gt = w_gt;
camera_pose_4.d_gt = d_gt;
r = [1 0 0 pi];
R = axang2rotm(r);
r = [0 0 1 pitch * pi];
R = axang2rotm(r) * R;

if is_rand_degree
    r = [0 1 0 (rand(1) * 0.5) * pi];
else
    r = [0 1 0 0.2 * pi];
end
R = axang2rotm(r) * R;
t = [0;0;scale]; 
camera_pose_4.gt_oritation = R;
camera_pose_4.gt_translation = t;
[Rot_Rows, trans_Rows] = linearEgoMotion(R,t,w_gt,d_gt,cameraParams);
[p2d_4,p3d_RS, flag_4] = RSWorld2Image(p3d_gt, Rot_Rows, trans_Rows, cameraParams);

p2d_4 = p2d_4 + randn(num_points, 2) * R_sig;
camera_pose_4.feature_point = p2d_4;



% camera five
w = 2 * (rand(3,1) - 0.5 * ones(3, 1));
d = 2 * (rand(3,1) - 0.5 * ones(3, 1));
w_gt = w / norm(w) * (pi * w_level / 180) * t_v;
d_gt = d / norm(d) * d_level * t_v;
camera_pose_5.w_gt = w_gt;
camera_pose_5.d_gt = d_gt;
r = [1 0 0 pi];
R = axang2rotm(r);
r = [0 0 1 -pitch * pi];
R = axang2rotm(r) * R;

if is_rand_degree
    r = [0 1 0 (rand(1) * 0.5 + 0.5) * pi];
else
    r = [0 1 0 0.8 * pi];
end
R = axang2rotm(r) * R;
t = [0;0;scale]; 
camera_pose_5.gt_oritation = R;
camera_pose_5.gt_translation = t;
[Rot_Rows, trans_Rows] = linearEgoMotion(R,t,w_gt,d_gt,cameraParams);
[p2d_5,p3d_RS, flag_5] = RSWorld2Image(p3d_gt, Rot_Rows, trans_Rows, cameraParams);
p2d_5 = p2d_5 + randn(num_points, 2) * R_sig;
camera_pose_5.feature_point = p2d_5;


flag = flag_1 | flag_2 | flag_3 | flag_4 | flag_5;

% ----------------------------- custom relative pose and triangulate -----------------------------
% E = estimateEssentialMatrix(p2d1 ,p2d2, cameraParams);
% [relativeOrientation,relativeLocation] = relativeCameraPose(E,cameraParams,p2d1,p2d2);
% 
% scale = norm(camera_pose.gt_oritation' * camera_pose.gt_translation - pose_set(1).gt_oritation' * pose_set(1).gt_translation);
% camera_pose.oritation = relativeOrientation * pose_set(1).gt_oritation;
% camera_pose.translation = ((pose_set(1).gt_translation' - scale * relativeLocation) * relativeOrientation')';
% 
% cameraMatrix1 = cameraMatrix(cameraParams,pose_set(1).gt_oritation',...
%      pose_set(1).gt_translation);
% cameraMatrix2 = cameraMatrix(cameraParams,camera_pose.oritation',...
%      camera_pose.translation);
     
%     t_interval = (2 * pi * config.radius / config.velocity) / (360 / config.delta_w);
%     w = rotationMatrixToVector(rotationVectorToMatrix(rs_camera_set.rotation(1,:))' * rotationVectorToMatrix(rs_camera_set.rotation(2,:)));
%     w_norm = norm(w) / t_interval;
%     w_normalized = w / norm(w);
%     w = w_normalized * w_norm / 720;
%     d_world_frame = rs_camera_set.translation(1,:) * rotationVectorToMatrix(rs_camera_set.rotation(1,:)) - rs_camera_set.translation(2,:) * rotationVectorToMatrix(rs_camera_set.rotation(2,:));
%     d_local = d_world_frame * rotationVectorToMatrix(rs_camera_set.rotation(1,:))';
%     d = - d_local / (t_interval * 720);
    %worldPoints = triangulate_RS(config.K,w',d',w',d',inlierPoints1,inlierPoints2,cameraMatrix1,cameraMatrix2);
% p3d = triangulate_RS(K,[0;0;0],[0;0;0],[0;0;0],[0;0;0],p2d1,p2d2,cameraMatrix1,cameraMatrix2);
% p3d_tri = triangulate(p2d1,p2d2,cameraMatrix1,cameraMatrix2);



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
camera_pose_4.oritation = relativeOrientation_3 * camera_pose_1.gt_oritation;
camera_pose_4.translation = ((camera_pose_1.translation' - scale_3 * relativeLocation_3) * relativeOrientation_3')';
camera_pose_5.oritation = relativeOrientation_4 * camera_pose_1.gt_oritation;
camera_pose_5.translation = ((camera_pose_1.translation' - scale_4 * relativeLocation_4) * relativeOrientation_4')';



tracks = [];
for i = 1:num_points_all
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
pose_pre_set(1) = camera_pose_1; 
pose_pre_set(2) = camera_pose_2;
pose_pre_set(3) = camera_pose_3;
pose_pre_set(4) = camera_pose_4;
pose_pre_set(5) = camera_pose_5;
for i = 1:camera_num
    pose_set(i) = pose_pre_set(i);
end


camera_set = pose_set;
pram.num_points_all = num_points_all;
p3d = xyzPoints;
pram.K_matrix = K;
pram.num_measurement = camera_num * num_points_all;
pram.num_points = num_points;
pram.num_camera = camera_num;
pram.H = H; 
pram.W = W; 
pram.fx = fx;
pram.fy = fy;
pram.cx = cx; 
pram.cy = cy;
end