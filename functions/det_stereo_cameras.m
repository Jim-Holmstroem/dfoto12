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
    I34 = [eye(3),zeros(3,1)];
    cam_centers(:,1) = [0,0,0,1]';
    cams(1:3,:) = K1*I34;
    [U,S,V] = svd(E);
    t = V(:,3);
    t = t/norm(t); %||t||=1
    
    cam_centers(:,2) = [t; 1]; % [X0,Y0,Z0,1]
    W = [0,-1,0;1,0,0;0,0,1];
    R1 = U*W*V';
    R2 = U*W'*V';

    R1 = det(R1)*R1;
    R2 = det(R2)*R2;

    cams(4:6,:) = K2*R1*[eye(3),t];
    P1 = det_model(cams, data(:,1)); %check where the first point ends up
    P1 = P1/P1(end); 
    P1_cam1 = I34*P1; %NOTE isn't this data ? M*P
    P1_cam2 = R1*[eye(3),t]*P1; %NOTE not the inner K, why?
    if( P1_cam1(end)>0 && P1_cam2(end)>0 ) %NOTE both in front of camera?
        return 
    end

    cams(4:6,:) = K2*R1*[eye(3),-t];
    P2 = det_model(cams, data(:,1)); %check where the first point ends up
    P2 = P2/P2(end); 
    P2_cam1 = I34*P2;
    P2_cam2 = R1*[eye(3),-t]*P2;
    if( P2_cam1(end)>0 && P2_cam2(end)>0 ) 
        return 
    end

    cams(4:6,:)= K2*R2*[eye(3),t];
    P3 = det_model(cams, data(:,1)); %check where the first point ends up
    P3 = P3/P3(end); 
    P3_cam1 = I34*P3;
    P3_cam2 = R2*[eye(3),t]*P3;
    if( P3_cam1(end)>0 && P3_cam2(end)>0 ) 
        return 
    end

    cams(4:6,:) = K2*R2*[eye(3),-t];
    P4 = det_model(cams, data(:,1)); %check where the first point ends up
    P4 = P4/P4(end); 
    P4_cam1 = I34*P4;
    P4_cam2 = R2*[eye(3),-t]*P4;
    if( P4_cam1(end)>0 && P4_cam2(end)>0 ) 
        return 
    end

