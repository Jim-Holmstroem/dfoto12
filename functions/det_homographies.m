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

%------------------------------
%
% FILL IN THIS PART
%
%------------------------------
            
    %NOTE points2 is the one with nans
    isnotnan = ~isnan(points2);
    points1 = reshape(points1(isnotnan),3,[]); %pick out the corresponding points1 (ref)
    points2 = reshape(points2(isnotnan),3,[]);

    xa = points1(1,:);
    ya = points1(2,:);
    xb = points2(1,:);
    yb = points2(2,:);

    sz = size(xa);

    alpha = [xb; yb; ones(sz); zeros(sz); zeros(sz); zeros(sz); -xb.*xa; -yb.*xa; -xa]'; 
    beta = [zeros(sz); zeros(sz); zeros(sz); xb; yb; ones(sz); -xb.*ya; -yb.*ya; -ya]'; 

    Q = [alpha; beta;];

    [U,S,V] = svd(Q);
    h = V(:,end);
    H = reshape(h, 3, 3)';

