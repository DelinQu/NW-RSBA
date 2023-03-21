function [F] = gsba_opter(x,cam_fir,pram, camera_set)

    num_points = pram.num_points;
    num_camera = pram.num_camera;
    num_measurement = pram.num_measurement;
    K = pram.K_matrix;
    F = zeros(2, num_measurement);
    f_x = K(1, 1);
    f_y = K(2, 2);
    u_0 = K(1, 3);
    v_0 = K(2, 3);
 %--------------------------------------------------------------------------   
    for i=1:num_camera 
        for j = 1:num_points
            pose_id = (i - 1) * 6 + 1 - 6;
            point_id = (num_camera  * 6) + (j - 1) * 3 + 1 - 6;   
            measurement_id = (i - 1) * num_points + j;
            if i ~= 1
                camera_pose = x(1, pose_id:pose_id + 5);
            else
                camera_pose = cam_fir(1, 1:6);
            end
            point_pose = x(1, point_id:point_id + 2);
            obs = camera_set(i).feature_point(j,:)';


            vi = obs(2);
            X = point_pose(1,1);
            Y = point_pose(1,2);
            Z = point_pose(1,3);
            fx = f_x;
            fy = f_y;
            rx = camera_pose(1,1);
            ry = camera_pose(1,2);
            rz = camera_pose(1,3);
            tx = camera_pose(1,4);
            ty = camera_pose(1,5);
            tz = camera_pose(1,6);
            u0 = u_0;
            v0 = v_0;
            u_obs = obs(1);
            v_obs = obs(2);
            F(1:2, measurement_id) = error_obs_only(X,Y,Z,fx,fy,rx,ry,rz,tx,ty,tz,u0,u_obs,v0,v_obs);

        end
    end
end