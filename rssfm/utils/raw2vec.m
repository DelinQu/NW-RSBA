function cam_point_pram = raw2vec(camera_set, p3d, p3d_gt,pram, is_output_gt)
    cam_point_pram = zeros(1,12 * pram.num_camera + 3 * pram.num_points_all);
    for i = 1:pram.num_camera
        id = (i - 1) * 12 + 1;
        if is_output_gt
            axang = rotm2axang(camera_set(i).gt_oritation);
        else
            axang = rotm2axang(camera_set(i).oritation);
        end
        if axang(4) == 0
            axang = [1 0 0 2 * pi];
        end
        axis_vec = axang(1:3) * axang(4);
        if is_output_gt
            cam_point_pram(id:id + 2) = axis_vec;
            cam_point_pram(id + 3: id + 5) = camera_set(i).gt_translation';
            cam_point_pram(id + 6:id + 8) = camera_set(i).w_gt';
            cam_point_pram(id + 9:id + 11) = camera_set(i).d_gt';
        else
            cam_point_pram(id:id + 2) = axis_vec;
            cam_point_pram(id + 3: id + 5) = camera_set(i).translation';
            cam_point_pram(id + 6:id + 8) = 1e-7 * ones(1,3);
            cam_point_pram(id + 9:id + 11) = 1e-7 * ones(1,3);

        end
    end
    for i = 1:pram.num_points_all
        id = pram.num_camera * 12 + (i - 1) * 3 + 1;
        if is_output_gt
            cam_point_pram(id: id + 2) = p3d_gt(i,:);
        else
            cam_point_pram(id: id + 2) = p3d(i,:);
        end
        
    end
end