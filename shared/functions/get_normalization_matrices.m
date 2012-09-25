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
%am_points = size(data,2);   
norm_mat = [];

for j = 1:am_cams
    pj = data(j*3-2:j*3,~isnan(data(3*j,:)));
    n = size(pj,2);
    c(:,j) = sum(pj,2)/n;
    d(j) = sum(sqrt(sum((pj-repmat(c(:,j),1,n)).^2,1)))/n;
    s2 = sqrt(2);
    norm_mat = [norm_mat; [s2/d(j) 0 -s2*c(1,j)/d(j);...
        0 s2/d(j) -s2*c(2,j)/d(j);...
        0 0 1]];
end

end





