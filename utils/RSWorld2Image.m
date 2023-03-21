function [p2d,p3d_RS,flag] = RSWorld2Image(p3d, Rot_Rows, trans_Rows, cameraParams)
flag = 0;
thr_dis = 0.6; 

%% data prepare 
% image size
width = cameraParams.IntrinsicMatrix(3,1) * 2;
height = cameraParams.IntrinsicMatrix(3,2) * 2;

% number of points
pNum = size(p3d,1);  % number of points

%% Rolling Shutter Projection 
p2d = []; % projected 2D points
p3d_RS = [];
for p_index = 1 : pNum
    closet_dist = thr_dis;
    bestPredictPoint = (Rot_Rows{1} *  p3d(p_index,:)' + trans_Rows{1});
    bestP3d_RS = bestPredictPoint;
    bestPredictPoint = cameraParams.IntrinsicMatrix' * bestPredictPoint;
    bestPredictPoint = bestPredictPoint(1:2,1) ./ bestPredictPoint(3,1);
    bestPredictPoint = bestPredictPoint';
    
    for row_index = 1:height
        R = Rot_Rows{row_index};
        % initial pose for current row 
        t = trans_Rows{row_index};
        
        % projection 3D point to 2D image 
        projectedPoints = cameraParams.IntrinsicMatrix' * (R * p3d(p_index,:)' + t);
        projectedPoints = projectedPoints(1:2)./ projectedPoints(3);
        projectedPoints = projectedPoints';
       
        % if right projected? 
        if projectedPoints(1,2) > row_index - thr_dis && projectedPoints(1,2) < row_index + thr_dis
            dist = abs(projectedPoints(1,2) - row_index);
            if dist < closet_dist
                bestPredictPoint = [projectedPoints(1,1),projectedPoints(1,2)];
                bestP3d_RS = R * p3d(p_index,:)' + t;
                closet_dist = dist;
            end
        end
    end
    if thr_dis == closet_dist
        flag = 1;
    end
     p2d = [p2d; bestPredictPoint];
     p3d_RS = [p3d_RS,bestP3d_RS];
end