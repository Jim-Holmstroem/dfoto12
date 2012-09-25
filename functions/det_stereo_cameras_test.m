test = load('test_det_stereo_cam');

[cams, cam_centers] = det_stereo_cameras(test.E,test.K1,test.K2,test.pts);
err_cams = test.cams-cams
err_centers = test.cam_centers-cam_centers

