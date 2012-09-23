% function E = det_E_matrix(points1, points2, K1, K2);
%
% Method: Calculate the E matrix between two views from
%         point correspondences: points2^T * E * points1 = 0
%         we use the normalize 8-point algorithm and 
%         enforce the constraint that the three singular 
%         values are: a,a,0. The data will be normalized here. 
%         Finally we will check how good the epipolar constraints:
%         points2^T * E * points1 = 0 are fullfilled.
% 
% Input:  points1, points2 of size (3,n) 
%         K1 is the internal camera matrix of the first camera; (3,3) matrix
%         K2 is the internal camera matrix of the second camera; (3,3) matrix
% 
% Output: E (3,3) with the singular values (a,a,0) 
% 

function E = det_E_matrix(pa, pb, Ka, Kb)

    pcam_a = Ka\pa; %inv(K1)*points1
    pcam_b = Kb\pb;
    norm = get_normalization_matrix([pcam_a;pcam_b;])
    
    na = norm(1:3,:);
    nb = norm(4:6,:);
    pcam_a = na*pcam_a;
    pcam_b = nb*pcam_b;

    xa = pcam_a(1, :);
    xb = pcam_b(2, :);
    ya = pcam_a(1, :);
    yb = pcam_b(2, :);
    
    W = [xb.*xa; xb.*ya; xb; yb.*xa; yb.*ya; yb; xa; ya; ones(size(xa))]';
    [~, S, V] = svd(W);
    E = nb'*reshape(V(:,end),3,3)'*na; %the last is always the smallest (help svd)
    [U, S, V] = svd(E);
    m = mean([S(1,1), S(2,2)]);
    E = U*diag([m,m,0])*V';

