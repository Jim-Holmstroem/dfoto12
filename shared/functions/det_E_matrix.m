% function E = det_E_matrix(p1, p2, K1, K2);
%
% Method: Calculate the E matrix between two views from
%         point correspondences: p2^T * E * p1 = 0
%         we use the normalize 8-point algorithm and 
%         enforce the constraint that the three singular 
%         values are: a,a,0. The data will be normalized here. 
%         Finally we will check how good the epipolar constraints:
%         p2^T * E * p1 = 0 are fullfilled.
% 
% Input:  p1, p2 of size (3,n) 
%         K1 is the internal camera matrix of the first camera; (3,3) matrix
%         K2 is the internal camera matrix of the second camera; (3,3) matrix
% 
% Output: E (3,3) with the singular values (a,a,0) 
% 

function E = det_E_matrix(p1, p2, K1, K2)
p1 = K1\p1;
p2 = K2\p2;
data = [p1; p2];
norm_mat = get_normalization_matrices(data);
p1 = norm_mat(1:3,:) * data(1:3,:);
p2 = norm_mat(4:6,:) * data(4:6,:);
noOfPoints = size(p1,2);
Y = zeros(9,noOfPoints);
for i=1:noOfPoints
    y = [p2(1,i)*p1(1,i), p2(1,i)*p1(2,i), p2(1,i), p2(2,i)*p1(1,i), ...
        p2(2,i)*p1(2,i), p2(2,i), p1(1,i), p1(2,i), 1]';
    Y(:,i) = y;
end

%Now we have e'Y = 0, which we can solve with SVD of Y
[~, S, V] = svd(Y*Y');
EV = diag(sqrt(S)); %|Eigenvalues| is sqrt of singular values of Y
[~, index] = min(abs(EV)); %Determines smallest eigenvalue
e = V(:,index);
E = reshape(e,3,3)';
E = norm_mat(4:6,:)'*E*norm_mat(1:3,:);
[U S V] = svd(E);
E = U * [(S(1,1)+S(2,2))/2 0 0;0 (S(1,1)+S(2,2))/2 0;0 0 0]*V';
end



