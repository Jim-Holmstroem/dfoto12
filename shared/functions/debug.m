clear all;
load('../debug/part1/test_norm_mat.mat');

reference_norm = norm_mat
our_norm = get_normalization_matrices(data)

diff = reference_norm - our_norm

%%
clear all;
load('../debug/part1/test_det_h.mat');

reference_H = H

our_H = det_homographies(p2,p1)

diff = reference_H - our_H