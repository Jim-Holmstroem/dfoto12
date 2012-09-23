% H = det_homographies(points1, points2)
%
% Method: Determines the mapping H * points1 = points2
% 
% Input:  points1, points2 are of the form (3,n) with 
%         n is the amount of points 
%         The points should be normalized for 
%         better performance 
% 
% Output: H (3,3) matrix 
%

function H = det_homographies(points1, points2)

%points1 from reference image
noOfPoints = size(points1,2);
alpha = zeros(noOfPoints,9);
beta = zeros(noOfPoints,9);
for i = 1:noOfPoints
    alpha(i,:) = [points2(1,i),points2(2,i),points2(3,i),0,0,0,...
        -points2(1,i)*points1(1,i),-points2(2,i)*points1(1,i),-points1(1,i)];
    beta(i,:) = [0,0,0,points2(1,i),points2(2,i),points2(3,i),...
        -points2(1,i)*points1(2,i),-points2(2,i)*points1(2,i),-points1(2,i)];
end

Q = [alpha; beta];
% [U S V] = svd(Q'*Q);
% EV = diag(sqrt(S));
[V D] = eig(Q'*Q);
EV = diag(D);
[~, index] = min(abs(EV));
h = V(:,index);
H = reshape(h,3,3)';
