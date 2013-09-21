% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 722.185193184120862 ; 721.310797334835797 ];

%-- Principal point:
cc = [ 323.916214422374708 ; 246.053469876996672 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.100514253346688 ; 0.107212471884339 ; -0.000157600870685 ; 0.001716463471191 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.218893408429492 ; 1.094277793150588 ];

%-- Principal point uncertainty:
cc_error = [ 2.800016894531061 ; 2.202553202087970 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.013661419504164 ; 0.092600135047584 ; 0.000865034080195 ; 0.001083741073164 ; 0.000000000000000 ];

%-- Image size:
nx = 640;
ny = 480;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 11;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 1.894419e+00 ; 1.923495e+00 ; 6.078192e-01 ];
Tc_1  = [ -2.002693e+02 ; -1.517219e+02 ; 6.452793e+02 ];
omc_error_1 = [ 3.094869e-03 ; 2.799755e-03 ; 4.983441e-03 ];
Tc_error_1  = [ 2.570743e+00 ; 2.028562e+00 ; 1.450792e+00 ];

%-- Image #2:
omc_2 = [ 1.640840e+00 ; 1.616264e+00 ; 1.059167e+00 ];
Tc_2  = [ -1.006191e+02 ; -1.387348e+02 ; 5.624389e+02 ];
omc_error_2 = [ 3.524514e-03 ; 2.841239e-03 ; 4.063544e-03 ];
Tc_error_2  = [ 2.220147e+00 ; 1.739006e+00 ; 1.200676e+00 ];

%-- Image #3:
omc_3 = [ -1.742604e+00 ; -1.872117e+00 ; 5.647958e-01 ];
Tc_3  = [ -2.052070e+02 ; -1.619371e+02 ; 7.613708e+02 ];
omc_error_3 = [ 3.198782e-03 ; 2.726986e-03 ; 4.642297e-03 ];
Tc_error_3  = [ 2.981196e+00 ; 2.363251e+00 ; 1.365314e+00 ];

%-- Image #4:
omc_4 = [ -1.470973e+00 ; -1.686719e+00 ; 8.609084e-01 ];
Tc_4  = [ -1.756238e+02 ; -1.655119e+02 ; 7.267801e+02 ];
omc_error_4 = [ 3.408566e-03 ; 2.986107e-03 ; 3.874843e-03 ];
Tc_error_4  = [ 2.858994e+00 ; 2.258901e+00 ; 1.160424e+00 ];

%-- Image #5:
omc_5 = [ -1.807197e+00 ; -1.919921e+00 ; -7.201698e-01 ];
Tc_5  = [ -1.802169e+02 ; -7.220283e+01 ; 5.853286e+02 ];
omc_error_5 = [ 2.093380e-03 ; 3.424245e-03 ; 5.010676e-03 ];
Tc_error_5  = [ 2.279150e+00 ; 1.828767e+00 ; 1.378912e+00 ];

%-- Image #6:
omc_6 = [ -1.600367e+00 ; -1.730977e+00 ; -9.449869e-01 ];
Tc_6  = [ -1.735890e+02 ; -2.001751e+00 ; 5.195574e+02 ];
omc_error_6 = [ 2.323496e-03 ; 3.605433e-03 ; 4.463834e-03 ];
Tc_error_6  = [ 2.014614e+00 ; 1.622149e+00 ; 1.274872e+00 ];

%-- Image #7:
omc_7 = [ 1.992416e+00 ; 1.795391e+00 ; -4.438995e-01 ];
Tc_7  = [ -2.101829e+02 ; -1.000244e+02 ; 9.046637e+02 ];
omc_error_7 = [ 2.498135e-03 ; 3.167698e-03 ; 5.598148e-03 ];
Tc_error_7  = [ 3.519893e+00 ; 2.789544e+00 ; 1.658904e+00 ];

%-- Image #8:
omc_8 = [ 1.560706e+00 ; 2.389772e+00 ; -7.525459e-01 ];
Tc_8  = [ -1.121556e+02 ; -1.525035e+02 ; 9.075000e+02 ];
omc_error_8 = [ 1.754846e-03 ; 3.725770e-03 ; 5.631293e-03 ];
Tc_error_8  = [ 3.523578e+00 ; 2.789315e+00 ; 1.352371e+00 ];

%-- Image #9:
omc_9 = [ -1.512262e+00 ; -1.741719e+00 ; -2.193219e-01 ];
Tc_9  = [ -2.115510e+02 ; -1.693629e+02 ; 6.521047e+02 ];
omc_error_9 = [ 2.414132e-03 ; 3.194714e-03 ; 4.217305e-03 ];
Tc_error_9  = [ 2.556578e+00 ; 2.043111e+00 ; 1.458115e+00 ];

%-- Image #10:
omc_10 = [ 1.843224e+00 ; 1.523389e+00 ; 2.135557e-01 ];
Tc_10  = [ -2.000346e+02 ; -1.194275e+02 ; 7.491576e+02 ];
omc_error_10 = [ 2.765572e-03 ; 2.830406e-03 ; 4.783420e-03 ];
Tc_error_10  = [ 2.925367e+00 ; 2.318795e+00 ; 1.559568e+00 ];

%-- Image #11:
omc_11 = [ -1.603347e+00 ; -2.110673e+00 ; -1.589243e+00 ];
Tc_11  = [ -1.743907e+01 ; -8.346847e+01 ; 5.448387e+02 ];
omc_error_11 = [ 2.079958e-03 ; 4.026298e-03 ; 5.261099e-03 ];
Tc_error_11  = [ 2.129320e+00 ; 1.679918e+00 ; 1.172416e+00 ];

