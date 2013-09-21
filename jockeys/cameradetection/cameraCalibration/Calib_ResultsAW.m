% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 718.126187560925473 ; 717.459614475409467 ];

%-- Principal point:
cc = [ 327.613841827922784 ; 254.170582769549014 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.114879366830427 ; 0.196399604113397 ; 0.001037475049048 ; 0.000882516124427 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.185273858185080 ; 1.115449342645891 ];

%-- Principal point uncertainty:
cc_error = [ 2.785102437339029 ; 2.078307462011728 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.011075361534016 ; 0.061379779166711 ; 0.000833901288020 ; 0.001077365519805 ; 0.000000000000000 ];

%-- Image size:
nx = 640;
ny = 480;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 13;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -1.561876e+00 ; -1.744205e+00 ; -1.117892e+00 ];
Tc_1  = [ -2.113949e+02 ; -1.288718e+02 ; 6.176606e+02 ];
omc_error_1 = [ 2.349704e-03 ; 3.293640e-03 ; 4.440644e-03 ];
Tc_error_1  = [ 2.444085e+00 ; 1.862683e+00 ; 1.586048e+00 ];

%-- Image #2:
omc_2 = [ -1.955794e+00 ; -2.223166e+00 ; -5.478420e-01 ];
Tc_2  = [ -1.953823e+02 ; -1.499344e+02 ; 6.982658e+02 ];
omc_error_2 = [ 2.372727e-03 ; 3.358775e-03 ; 5.966429e-03 ];
Tc_error_2  = [ 2.754073e+00 ; 2.081776e+00 ; 1.678990e+00 ];

%-- Image #3:
omc_3 = [ 1.909175e+00 ; 2.162003e+00 ; 6.851844e-02 ];
Tc_3  = [ -1.905829e+02 ; -1.639688e+02 ; 6.603700e+02 ];
omc_error_3 = [ 2.461917e-03 ; 3.143994e-03 ; 5.642271e-03 ];
Tc_error_3  = [ 2.603291e+00 ; 1.951036e+00 ; 1.468461e+00 ];

%-- Image #4:
omc_4 = [ 1.713575e+00 ; 1.839491e+00 ; -5.946199e-01 ];
Tc_4  = [ -1.831786e+02 ; -1.673111e+02 ; 6.827098e+02 ];
omc_error_4 = [ 1.946171e-03 ; 3.305264e-03 ; 4.757693e-03 ];
Tc_error_4  = [ 2.681781e+00 ; 2.007415e+00 ; 1.349567e+00 ];

%-- Image #5:
omc_5 = [ 1.686380e+00 ; 1.984539e+00 ; 6.986490e-01 ];
Tc_5  = [ -1.799639e+02 ; -1.727305e+02 ; 5.572443e+02 ];
omc_error_5 = [ 2.906682e-03 ; 2.844467e-03 ; 4.315202e-03 ];
Tc_error_5  = [ 2.253510e+00 ; 1.676488e+00 ; 1.352822e+00 ];

%-- Image #6:
omc_6 = [ 1.275068e+00 ; 1.705878e+00 ; 1.148425e+00 ];
Tc_6  = [ -6.329136e+01 ; -1.503739e+02 ; 4.944632e+02 ];
omc_error_6 = [ 3.373278e-03 ; 3.070953e-03 ; 3.508390e-03 ];
Tc_error_6  = [ 1.960346e+00 ; 1.451954e+00 ; 1.114599e+00 ];

%-- Image #7:
omc_7 = [ -1.798829e+00 ; -2.142674e+00 ; 4.955785e-01 ];
Tc_7  = [ -1.551276e+02 ; -1.439269e+02 ; 7.379254e+02 ];
omc_error_7 = [ 2.822944e-03 ; 2.831225e-03 ; 4.969376e-03 ];
Tc_error_7  = [ 2.861673e+00 ; 2.156209e+00 ; 1.322391e+00 ];

%-- Image #8:
omc_8 = [ -1.612079e+00 ; -1.783358e+00 ; 8.292014e-01 ];
Tc_8  = [ -1.462237e+02 ; -1.004152e+02 ; 9.231893e+02 ];
omc_error_8 = [ 3.160758e-03 ; 3.005582e-03 ; 4.175023e-03 ];
Tc_error_8  = [ 3.574153e+00 ; 2.695927e+00 ; 1.411480e+00 ];

%-- Image #9:
omc_9 = [ 1.423664e+00 ; 2.306865e+00 ; 1.366210e+00 ];
Tc_9  = [ -1.376520e+02 ; -1.687558e+02 ; 6.348730e+02 ];
omc_error_9 = [ 3.845639e-03 ; 2.720819e-03 ; 4.507519e-03 ];
Tc_error_9  = [ 2.534142e+00 ; 1.903670e+00 ; 1.566089e+00 ];

%-- Image #10:
omc_10 = [ -1.664055e+00 ; -1.644643e+00 ; 1.301910e-01 ];
Tc_10  = [ -1.692994e+02 ; -1.264335e+02 ; 8.233499e+02 ];
omc_error_10 = [ 2.509511e-03 ; 3.075722e-03 ; 4.317026e-03 ];
Tc_error_10  = [ 3.197646e+00 ; 2.395013e+00 ; 1.501523e+00 ];

%-- Image #11:
omc_11 = [ 1.600466e+00 ; 2.335405e+00 ; -1.348926e+00 ];
Tc_11  = [ -1.668591e+02 ; -1.326998e+02 ; 8.649319e+02 ];
omc_error_11 = [ 1.569295e-03 ; 3.850550e-03 ; 5.447012e-03 ];
Tc_error_11  = [ 3.371531e+00 ; 2.556362e+00 ; 1.349581e+00 ];

%-- Image #12:
omc_12 = [ 1.723408e+00 ; 1.512251e+00 ; 1.957114e-01 ];
Tc_12  = [ -2.074099e+02 ; -1.789855e+02 ; 7.331400e+02 ];
omc_error_12 = [ 2.579015e-03 ; 3.133761e-03 ; 4.248562e-03 ];
Tc_error_12  = [ 2.887638e+00 ; 2.156511e+00 ; 1.702414e+00 ];

%-- Image #13:
omc_13 = [ 1.620948e+00 ; 2.264950e+00 ; 1.259036e+00 ];
Tc_13  = [ -1.429986e+02 ; -1.554664e+02 ; 7.302265e+02 ];
omc_error_13 = [ 3.914733e-03 ; 2.657576e-03 ; 4.781359e-03 ];
Tc_error_13  = [ 2.889771e+00 ; 2.167491e+00 ; 1.728607e+00 ];

