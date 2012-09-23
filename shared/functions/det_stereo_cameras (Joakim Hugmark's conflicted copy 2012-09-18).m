% function [cams, cam_centers] = det_stereo_cameras(E, K1, K2, data); 
%
% Method: Calculate the first and second camera matrix. 
%         The second camera matrix is unique up to scale. 
%         The essential matrix and 
%         the internal camera matrices are known. Furthermore one 
%         point is needed in order solve the ambiguity in the 
%         second camera matrix.
% 
% Input:  E - essential matrix with the singular values (a,a,0) 
%         K1 - internal matrix of the first camera; (3,3) matrix
%         K2 - internal matrix of the second camera; (3,3) matrix
%         data - (6,1) matrix, of 2 points in camera one and two 
% 
% Output: cams (6,4), where cams(1:3,:) is the first and 
%                     cams(4:6,:) is the second camera      
%         cam_centers(4,2) where (:,1) is the first and (:,2) 
%                          the second camera center
%

function [cams, cam_centers] = det_stereo_cameras(E, K1, K2, data)

%Set camera 1 as unit camera
cam_centers(:,1) = [0 0 0 1]';
cams(1:3,:) = K1*[1 0 0 0; 0 1 0 0; 0 0 1 0];

[U S V] = svd(E);

%Find translation vector, which is given by Et=0
t = V(:,3);
t = t/norm(t)
cam_centers(:,2) = [t; 1]; 

%Find possible rotation matrices
W = [0 -1 0; 1 0 0; 0 0 1];
R1 = U*W*V';
R2 = U*W'*V';

%If rotation matrix' determinant is -1, we must multiply it with -1
R1 = det(R1)*R1;
R2 = det(R2)*R2;

%Determine if R1 or R2 should be used, and if t or -t should be used.
%R1 and t
cams(4:6,:) = K2*R1*[eye(3),t]
P1 = det_model(cams,data(:,1))

%R1 and -t
cams(4:6,:) = K2*R1*[eye(3),-t]
P2 = det_model(cams,data(:,1))

%R2 and t
cams(4:6,:) = K2*R2*[eye(3),t]
P3 = det_model(cams,data(:,1))

%R2 and -t
cams(4:6,:) = K2*R2*[eye(3),-t]
P4 = det_model(cams,data(:,1))

%Soo, which do we choose? Apparently, The second one is correct, at least
%for the debug-point, but why?