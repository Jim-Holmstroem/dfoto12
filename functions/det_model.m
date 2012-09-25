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
    cama = cam(1:3,:);
    camb = cam(4:6,:);

    npoints = size(data,2);
    model = zeros(4,npoints);

    for i = 1:npoints %Determine each point, there is no other way, must use forloop
        pa = data(1:2,i);
        pb = data(4:5,i);
        W = [pa*cama(3,:);pb*camb(3,:)]-cam([1,2,4,5],:);
        [U, S, V] = svd(W);
        model(:,i) = V(:,end);
    end
