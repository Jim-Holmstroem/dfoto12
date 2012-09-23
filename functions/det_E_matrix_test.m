test = load('test_det_e_mat.mat');

Erel = test.E
E = det_E_matrix(test.d1,test.d2,test.K1,test.K2)

