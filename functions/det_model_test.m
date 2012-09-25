test = load('test_det_model');
isgood = max(max(abs(test.model-det_model(test.cams_norm, test.data_norm))))<1.0e-10
