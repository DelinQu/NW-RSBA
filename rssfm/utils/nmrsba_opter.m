function [F,J] = nmrsba_opter(x,cam_fir,pram, camera_set)
    num_points = pram.num_points;
    num_camera = pram.num_camera;
    num_measurement = pram.num_measurement;
    K = pram.K_matrix;
    F = zeros(2, num_measurement);
    J = zeros(2 * num_measurement, num_camera * 12 + num_points * 3 - 6);
    f_x = K(1, 1);
    f_y = K(2, 2);
    u_0 = K(1, 3);
    v_0 = K(2, 3);
 %--------------------------------------------------------------------------   
    for i=1:num_camera 
        for j = 1:num_points
            pose_id = (i - 1) * 12 + 1 - 6;
            point_id = (num_camera  * 12) + (j - 1) * 3 + 1 - 6;   
            measurement_id = (i - 1) * num_points + j;
            if i ~= 1
                camera_pose = x(1, pose_id:pose_id + 5);
            else
                camera_pose = cam_fir(1, 1:6);
            end
            rs_camera_pose = x(1, pose_id + 6:pose_id + 11);
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
            w_x = rs_camera_pose(1,1);
            w_y = rs_camera_pose(1,2);
            w_z = rs_camera_pose(1,3);
            d_x = rs_camera_pose(1,4);
            d_y = rs_camera_pose(1,5);
            d_z = rs_camera_pose(1,6);
            u0 = u_0;
            v0 = v_0;
            u_obs = obs(1);
            v_obs = obs(2);
            j_id = (measurement_id - 1) * 2 + 1;
            F(1:2, measurement_id) = error_obs_normal(X,Y,Z,d_x,d_y,d_z,fx,fy,rx,ry,rz,tx,ty,tz,u0,u_obs,v0,v_obs,vi,w_x,w_y,w_z);


            if i ~= 1
                tmp_j =  J_normal_func(X,Y,Z,d_x,d_y,d_z,fy,rx,ry,rz,tx,ty,tz,v0,vi,w_x,w_y,w_z);
                J(j_id:j_id + 1,pose_id:pose_id + 11) = tmp_j(:,1:12);
                J(j_id:j_id + 1, point_id:point_id + 2) = tmp_j(:,13:15);

            else
                point_id = (num_camera  * 12) + (j - 1) * 3 + 1 - 6;
                j_id = (j - 1) * 2 + 1;
                
                tmp_j = J_normal_func(X,Y,Z,d_x,d_y,d_z,fy,rx,ry,rz,tx,ty,tz,v0,vi,w_x,w_y,w_z);
                J(j_id:j_id + 1,pose_id + 6:pose_id + 11) = tmp_j(:,7:12);
                J(j_id:j_id + 1,point_id:point_id + 2) = tmp_j(:,13:15);
            end


         
        end
    end


end