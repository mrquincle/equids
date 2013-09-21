% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 718.580889674630271 ; 717.662440678194230 ];

%-- Principal point:
cc = [ 326.142929841106252 ; 252.234578729858242 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.101092690730525 ; 0.158474128759595 ; 0.001620254910150 ; 0.000047660226809 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.477266681065443 ; 1.499185487371215 ];

%-- Principal point uncertainty:
cc_error = [ 3.810906858234234 ; 2.738356737150276 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.014555447721921 ; 0.091875523236361 ; 0.001096020520324 ; 0.001426195214560 ; 0.000000000000000 ];

%-- Image size:
nx = 640;
ny = 480;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 10;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 1.786723e+00 ; 1.858424e+00 ; -9.055354e-01 ];
Tc_1  = [ -1.895170e+02 ; -6.527952e+01 ; 8.943238e+02 ];
omc_error_1 = [ 2.398835e-03 ; 4.613174e-03 ; 6.192785e-03 ];
Tc_error_1  = [ 4.743449e+00 ; 3.432669e+00 ; 1.849594e+00 ];

%-- Image #2:
omc_2 = [ -2.212389e+00 ; -2.175053e+00 ; 3.936809e-01 ];
Tc_2  = [ -2.086167e+02 ; -1.439849e+02 ; 7.194749e+02 ];
omc_error_2 = [ 4.378197e-03 ; 2.975978e-03 ; 6.897531e-03 ];
Tc_error_2  = [ 3.838693e+00 ; 2.769591e+00 ; 1.791394e+00 ];

%-- Image #3:
omc_3 = [ -1.651330e+00 ; -1.739605e+00 ; -1.057575e+00 ];
Tc_3  = [ -1.939220e+02 ; -8.607410e+01 ; 5.940900e+02 ];
omc_error_3 = [ 2.702440e-03 ; 4.799698e-03 ; 5.661761e-03 ];
Tc_error_3  = [ 3.182646e+00 ; 2.326807e+00 ; 1.800689e+00 ];

%-- Image #4:
omc_4 = [ -1.531803e+00 ; -1.556789e+00 ; -1.113189e+00 ];
Tc_4  = [ -2.141134e+02 ; -2.868245e+01 ; 5.778881e+02 ];
omc_error_4 = [ 3.076770e-03 ; 4.927879e-03 ; 5.201678e-03 ];
Tc_error_4  = [ 3.073682e+00 ; 2.265744e+00 ; 1.842730e+00 ];

%-- Image #5:
omc_5 = [ -2.208889e+00 ; -1.965992e+00 ; -4.712310e-01 ];
Tc_5  = [ -2.657152e+02 ; -1.887638e+02 ; 7.763866e+02 ];
omc_error_5 = [ 3.857849e-03 ; 3.875236e-03 ; 7.435732e-03 ];
Tc_error_5  = [ 4.243581e+00 ; 3.093463e+00 ; 2.275190e+00 ];

%-- Image #6:
omc_6 = [ -2.014741e+00 ; -1.939723e+00 ; -1.402295e+00 ];
Tc_6  = [ -8.491342e+01 ; -1.314727e+02 ; 5.901406e+02 ];
omc_error_6 = [ 2.096414e-03 ; 4.957982e-03 ; 7.011445e-03 ];
Tc_error_6  = [ 3.193133e+00 ; 2.274612e+00 ; 1.686004e+00 ];

%-- Image #7:
omc_7 = [ -1.801764e+00 ; -1.511442e+00 ; -2.691667e-01 ];
Tc_7  = [ -1.404530e+02 ; -1.484384e+02 ; 8.264067e+02 ];
omc_error_7 = [ 3.112787e-03 ; 4.221531e-03 ; 5.312144e-03 ];
Tc_error_7  = [ 4.414745e+00 ; 3.157082e+00 ; 1.887352e+00 ];

%-- Image #8:
omc_8 = [ -1.469964e+00 ; -1.598045e+00 ; 7.613444e-01 ];
Tc_8  = [ -9.408895e+01 ; -1.757554e+02 ; 9.049559e+02 ];
omc_error_8 = [ 4.429165e-03 ; 3.844373e-03 ; 4.570908e-03 ];
Tc_error_8  = [ 4.819081e+00 ; 3.453257e+00 ; 1.684353e+00 ];

%-- Image #9:
omc_9 = [ 2.045799e+00 ; 1.849965e+00 ; 7.231892e-01 ];
Tc_9  = [ -1.895875e+02 ; -1.759801e+02 ; 7.144726e+02 ];
omc_error_9 = [ 4.712487e-03 ; 3.534369e-03 ; 6.034522e-03 ];
Tc_error_9  = [ 3.889845e+00 ; 2.777517e+00 ; 2.061289e+00 ];

%-- Image #10:
omc_10 = [ 1.748664e+00 ; 1.367109e+00 ; 1.245503e+00 ];
Tc_10  = [ -7.724325e+01 ; -1.891012e+02 ; 6.905323e+02 ];
omc_error_10 = [ 5.331005e-03 ; 3.705416e-03 ; 4.640434e-03 ];
Tc_error_10  = [ 3.757551e+00 ; 2.657038e+00 ; 2.012817e+00 ];

