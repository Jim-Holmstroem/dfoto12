% function model = det_model(cam, data)
%
% Method: Determines the 3D model points by triangulation
%         of a stereo camera system. We assume that the data 
%         is already normalized 
% 
% Input: data (6,n) matrix
%        cam (6,4) matrix
% 
% Output: model (4,n matrix of all n points)
%

function model = det_model(cam, data)

noOfPoints = size(data,2);
ma11 = cam(1,1);
ma12 = cam(1,2);
ma13 = cam(1,3);
ma14 = cam(1,4);
ma21 = cam(2,1);
ma22 = cam(2,2);
ma23 = cam(2,3);
ma24 = cam(2,4);
ma31 = cam(3,1);
ma32 = cam(3,2);
ma33 = cam(3,3);
ma34 = cam(3,4);
mb11 = cam(4,1);
mb12 = cam(4,2);
mb13 = cam(4,3);
mb14 = cam(4,4);
mb21 = cam(5,1);
mb22 = cam(5,2);
mb23 = cam(5,3);
mb24 = cam(5,4);
mb31 = cam(6,1);
mb32 = cam(6,2);
mb33 = cam(6,3);
mb34 = cam(6,4);

model = zeros(4,noOfPoints);
%SEE EQUATION ON PAGE 24 IN THE LECTURE NOTES!

for i = 1:noOfPoints
    xa = data(1,i);
    ya = data(2,i);
    xb = data(4,i);
    yb = data(5,i);
    W = [xa*ma31-ma11, xa*ma32-ma12, xa*ma33-ma13, xa*ma34-ma14;...
        ya*ma31-ma21, ya*ma32-ma22, ya*ma33-ma23, ya*ma34-ma24;...
        xb*mb31-mb11, xb*mb32-mb12, xb*mb33-mb13, xb*mb34-mb14;...
        yb*mb31-mb21, yb*mb32-mb22, yb*mb33-mb23, yb*mb34-mb24];
    %We now have W*h = 0, solve with SVD
    %[U S V] = svd(W'*W);
    %EV = diag(sqrt(S));
    [V S] = eig(W'*W);
    EV = diag(S);
    [~, index] = min(abs(EV));
    model(:,i) = V(:,index);%*sign(V(4,index)*-1);
end

