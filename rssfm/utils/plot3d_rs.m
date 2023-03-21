function[] = plot3d_rs(gt_camera_param, refined_param_jacobian, p3d_gt, p3d, pram, color, color_point)


Scene3DFigure = figure;

for i = 1:pram.num_camera
    id = (i - 1) * 12 + 1;
    
    gt_rotation = gt_camera_param(id : id + 2);
    jacobian_rotation = refined_param_jacobian(id : id + 2);

    
    gt_rotation = axang2rotm([gt_rotation / norm(gt_rotation), norm(gt_rotation)]);
    jacobian_rotation = axang2rotm([jacobian_rotation / norm(jacobian_rotation), norm(jacobian_rotation)]);
       
    gt_translation = gt_camera_param(id + 3: id + 5)';
    jacobian_translation = refined_param_jacobian(id + 3: id + 5)';

    
    cam = plotCamera('Location',  -inv(gt_rotation) * gt_translation,'Orientation',(gt_rotation),'Opacity',0, 'Size', 1, 'color',"black" );
    hold on
    cam1 = plotCamera('Location',  -inv(jacobian_rotation) * jacobian_translation,'Orientation',(jacobian_rotation),'Opacity',0, 'Size', 1, 'color', color);
    hold on

end
grid on
axis equal

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

plot3(p3d_gt(:,1), p3d_gt(:,2), p3d_gt(:,3), 'black*');
hold on
plot3(p3d(:,1), p3d(:,2), p3d(:,3), color_point);
hold off
end