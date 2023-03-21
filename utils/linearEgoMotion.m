function [Rot_Rows, trans_Rows] = linearEgoMotion(R0,t0,w,d, cameraParams)

%% Data Prepare 
% image size
width = cameraParams.IntrinsicMatrix(3,1) * 2;
height = cameraParams.IntrinsicMatrix(3,2) * 2;

%% Ego-motion matrices generation 
Rot_Rows = {};
trans_Rows = {};
last_r = eye(3);
last_t = t0;
if norm(w) == 0
    one_r = eye(3);
else
    one_r = axang2rotm([(w / norm(w))' norm(w)]);
end


for row_index = 1 : height
    last_t = (d + last_t);
    last_r = one_r * last_r;
    cur_t = last_t;
    cur_r = last_r * R0;
    Rot_Rows{row_index} = cur_r; 
    trans_Rows{row_index} = cur_t;
end