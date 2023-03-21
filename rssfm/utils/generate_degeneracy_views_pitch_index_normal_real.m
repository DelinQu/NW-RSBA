function [camera_set_mid, p3d] = generate_degeneracy_views_pitch_index_normal_real(camera_set, p3d_in, pram)


camera_set_mid = camera_set;
cameraParams = cameraParameters('IntrinsicMatrix',pram.K_matrix');
num_points = pram.num_points;
num_camera = pram.num_camera;
p3d = p3d_in;

for i = 1:num_camera
R = camera_set_mid(i).gt_oritation;
t = camera_set_mid(i).gt_translation;
w_gt = camera_set_mid(i).w_gt;
d_gt = camera_set_mid(i).d_gt;
[Rot_Rows, trans_Rows] = linearEgoMotion(R,t,w_gt,d_gt,cameraParams);
camera_set_mid(i).gt_oritation = Rot_Rows{pram.cy};
camera_set_mid(i).gt_translation = trans_Rows{pram.cy};
camera_set_mid(i).w_gt = w_gt * pram.fy;
camera_set_mid(i).d_gt = d_gt * pram.fy;
end
camera_set_mid(1).oritation = camera_set_mid(1).gt_oritation;
camera_set_mid(1).translation = camera_set_mid(1).gt_translation;

end