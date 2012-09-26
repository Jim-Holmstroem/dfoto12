% function [error_average, error_max] = check_reprojection_error(data, cam, model)
%
% Method: Evaluates average and maximum error 
%         between the reprojected image points (cam*model) and the 
%         given image points (data), i.e. data = cam * model 
%
% Input:  data (3*m,n) matrix with m cameras and n points 
%         cam  (3*m,4) matrix 
%         model (4,n) matrix
%       
% Output: the average error (error_average) and maximum error (error_max)
%      

function [error_average, error_max] = check_reprojection_error(data, cam, model)
    reproj_data = cam*model;
    
    for i = 1:3:size(cam,1) %fugly but have 2 do it
        reproj_data_norm(i:(i+2),:) = norm_points_to_one(reproj_data(i:(i+2),:));
    end

    err = data - reproj_data_norm;
    error_average = mean(mean(abs(err)));
    error_max = max(max(abs(err)));

