% function [norm_mat] = get_normalization_matrix(data);   
%
% Method: get all normalization matrices.  
%         It is: point_norm = norm_matrix * point. The norm_points 
%         have centroid 0 and average distance = sqrt(2))
% 
% Input: data (3*m,n) (not normalized) the data may have NaN values 
%        for m cameras and n points 
%        
% Output: norm_mat is a 3*m,3 matrix, which consists of all 
%         normalization matrices matrices, i.e. norm_mat(1:3,:) is the 
%         matrix for camera 1 
%

function [norm_mat] = get_normalization_matrix(data);   

% get Info 
am_cams = size(data,1)/3;  
am_points = size(data,2);   

for hi1 = 1:am_cams
    d = data(hi1*3-2:hi1*3,:);
    d = reshape(d(~isnan(d)),3,[]);
    centroid = mean(d,2);
    n = size(d,2);
    distance = sum(sqrt(sum((d-repmat(centroid,[1,n])).^2)))/n; %NOTE should have been sum(norm(d-repmat(centroid))), but norm is retarded in matlab
    norm_mat(hi1*3-2:hi1*3, :) = [sqrt(2)./distance,                0,-sqrt(2)*centroid(1)/distance; ...
                                                  0,sqrt(2)./distance,-sqrt(2)*centroid(2)/distance; ...
                                                  0,                0,                            1];
end
% as a first test
if false
    for hi1 = 1:am_cams
      norm_mat(hi1*3-2:hi1*3,:) = eye(3);
    end
end
