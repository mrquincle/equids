#include "CTransformation.h"
#include <stdio.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_linalg.h>
#include <stdio.h>
#include <iostream>
CTransformation::CTransformation(int widthi, int heighti, float diam,
		bool fullUnbarreli) {
	transformType = TRANSFORM_NONE;
	fullUnbarrel = fullUnbarreli;
	width = widthi;
	height = heighti;
	char dummy[1000];
	FILE* file;
	if (!(file = fopen("./Calib_Results.m", "r"))) {
		std::cout << "CANNOT FIND CALIBRESULTS" << std::endl;
	}
	trackedObjectDiameter = diam;

	while (feof(file) == false) {
		fscanf(file, "%s", dummy);
		if (strcmp(dummy, "fc") == 0) {
			for (int j = 0; j < 10; j++) {
				fscanf(file, "%s\n", dummy);
				if (j == 2)
					fc[0] = atof(dummy);
				if (j == 4)
					fc[1] = atof(dummy);
			}
		}
		if (strcmp(dummy, "cc") == 0) {
			for (int j = 0; j < 10; j++) {
				fscanf(file, "%s\n", dummy);
				if (j == 2)
					cc[0] = atof(dummy);
				if (j == 4)
					cc[1] = atof(dummy);
			}
		}
		if (strcmp(dummy, "kc") == 0) {
			for (int j = 0; j < 12; j++) {
				fscanf(file, "%s\n", dummy);
				if (j > 1 && j % 2 == 0)
					kc[j / 2] = atof(dummy);
			}
		}
	}
	kc[0] = 1.0;
//	for (int i = 0;i<2;i++) printf("%05f,",fc[i]);
//	for (int i = 0;i<2;i++) printf("%05f,",cc[i]);
//	for (int i = 0;i<6;i++) printf("%05f,",kc[i]);
	unbarrelInitialized = false;

	if (fullUnbarrel) {
		unbarrelInitialized = true;
		float ix, iy;
		float gx, gy, ux, uy;
		xArray = (float*) malloc(width * height * sizeof(float));
		yArray = (float*) malloc(width * height * sizeof(float));
		gArrayX = (float*) malloc(width * height * sizeof(float));
		gArrayY = (float*) malloc(width * height * sizeof(float));
		pArray = (int*) malloc(width * height * sizeof(int) * 4);

		for (int x = 0; x < width; x++) {
			for (int y = 0; y < height; y++) {
				xArray[y * width + x] = barrelX(x, y);
				yArray[y * width + x] = barrelY(x, y);
				if (xArray[y * width + x] < 0
						|| xArray[y * width + x] > (width - 1)
						|| yArray[y * width + x] < 0
						|| yArray[y * width + x] > (height - 1)) {
					xArray[y * width + x] = 0;
					yArray[y * width + x] = 0;
				}
				ux = trunc(xArray[y * width + x]);
				uy = trunc(yArray[y * width + x]);
				gx = xArray[y * width + x] - ux;
				gy = yArray[y * width + x] - uy;
				ix = (int) ux;
				iy = (int) uy;
				pArray[y * width + x] = width * iy + ix;
				gArrayX[y * width + x] = gx;
				gArrayY[y * width + x] = gy;
			}
		}
	}
	loadCalibration("default.cal");
}

CTransformation::~CTransformation() {
	if (unbarrelInitialized) {
		free(xArray);
		free(yArray);
		free(gArrayX);
		free(gArrayY);
		free(pArray);
	}
}

void CTransformation::setTrackedObjectDiameter(float diam) {
	this->trackedObjectDiameter = diam;
}

float CTransformation::barrelX(float x, float y) {
	x = (x - cc[0]) / fc[0];
	y = (y - cc[1]) / fc[1];
	float cx, dx;
	float r = x * x + y * y;
	dx = 2 * kc[3] * x * y + kc[4] * (r + 2 * x * x);
	cx = (1 + kc[1] * r + kc[2] * r * r + kc[5] * r * r * r) * x + dx;
	cx = cx * fc[0] + cc[0];
	return cx;
}

float CTransformation::barrelY(float x, float y) {
	x = (x - cc[0]) / fc[0];
	y = (y - cc[1]) / fc[1];
	float cy, dy;
	float r = x * x + y * y;
	dy = 2 * kc[4] * x * y + kc[3] * (r + 2 * y * y);
	cy = (1 + kc[1] * r + kc[2] * r * r + kc[5] * r * r * r) * y + dy;
	cy = cy * fc[1] + cc[1];
	return cy;
}

float CTransformation::unbarrelX(float x, float y) {
	if (fullUnbarrel)
		return x;
	float ix, iy, dx, dy, r, rad;
	ix = x = (x - cc[0]) / fc[0];
	iy = y = (y - cc[1]) / fc[1];
	for (int i = 0; i < 5; i++) {
		r = x * x + y * y;
		dx = 2 * kc[3] * x * y + kc[4] * (r + 2 * x * x);
		dy = 2 * kc[4] * x * y + kc[3] * (r + 2 * y * y);
		rad = 1 + kc[1] * r + kc[2] * r * r + kc[5] * r * r * r;
		x = (ix - dx) / rad;
		y = (iy - dy) / rad;
	}
	return (x * fc[0] + cc[0]);
}

float CTransformation::unbarrelY(float x, float y) {
	if (fullUnbarrel)
		return y;
	float ix, iy, dx, dy, r, rad;
	ix = x = (x - cc[0]) / fc[0];
	iy = y = (y - cc[1]) / fc[1];
	for (int i = 0; i < 5; i++) {
		r = x * x + y * y;
		dx = 2 * kc[3] * x * y + kc[4] * (r + 2 * x * x);
		dy = 2 * kc[4] * x * y + kc[3] * (r + 2 * y * y);
		rad = 1 + kc[1] * r + kc[2] * r * r + kc[5] * r * r * r;
		x = (ix - dx) / rad;
		y = (iy - dy) / rad;
	}
	return (y * fc[1] + cc[1]);
}

void CTransformation::transformXY(float *ax, float *ay) {
	float x, y, ix, iy, dx, dy, r, rad;
	*ax = ix = x = (*ax - cc[0]) / fc[0];
	*ay = iy = y = (*ay - cc[1]) / fc[1];
	if (fullUnbarrel)
		return;
	for (int i = 0; i < 5; i++) {
		r = x * x + y * y;
		dx = 2 * kc[3] * x * y + kc[4] * (r + 2 * x * x);
		dy = 2 * kc[4] * x * y + kc[3] * (r + 2 * y * y);
		rad = 1 + kc[1] * r + kc[2] * r * r + kc[5] * r * r * r;
		x = (ix - dx) / rad;
		y = (iy - dy) / rad;
	}
	*ax = x;
	*ay = y;
}

float CTransformation::transformX(float xc, float yc) {
	return (unbarrelX(xc, yc) - cc[0]) / fc[0];
}

float CTransformation::transformY(float xc, float yc) {
	return (unbarrelY(xc, yc) - cc[1]) / fc[1];
}

void CTransformation::unbarrel(unsigned char *dst, unsigned char *src) {
	src[0] = src[1] = src[2] = 255;
	if (fullUnbarrel) {
		float gx, gy;
		for (int p = 0; p < width * (height - 1); p++) {
			gx = gArrayX[p];
			gy = gArrayY[p];
			dst[3 * p] = src[3 * pArray[p]] * (1 - gx) * (1 - gy)
					+ src[3 * pArray[p] + 3] * gx * (1 - gy)
					+ src[3 * pArray[p] + width * 3] * (1 - gx) * gy
					+ src[3 * pArray[p] + (width + 1) * 3] * gx * gy;
			dst[3 * p + 1] = src[3 * pArray[p] + 1] * (1 - gx) * (1 - gy)
					+ src[3 * pArray[p] + 3 + 1] * gx * (1 - gy)
					+ src[3 * pArray[p] + width * 3 + 1] * (1 - gx) * gy
					+ src[3 * pArray[p] + (width + 1) * 3 + 1] * gx * gy;
			dst[3 * p + 2] = src[3 * pArray[p] + 2] * (1 - gx) * (1 - gy)
					+ src[3 * pArray[p] + 3 + 2] * gx * (1 - gy)
					+ src[3 * pArray[p] + width * 3 + 2] * (1 - gx) * gy
					+ src[3 * pArray[p] + (width + 1) * 3 + 2] * gx * gy;
		}
	} else {
		fprintf(stdout, "Image unbarrel was not enabled\n");
	}
}

STrackedObject CTransformation::transform2D(STrackedObject o) {
	STrackedObject r;
	r.x = hom[0] * o.x + hom[1] * o.y + hom[2];
	r.y = hom[3] * o.x + hom[4] * o.y + hom[5];
	r.z = hom[6] * o.x + hom[7] * o.y + hom[8];
	r.x = r.x / r.z;
	r.y = r.y / r.z;
	r.z = 0;
	//printf("%.3f %.3f\n",r.x,r.y);
	r.error = establishError(r);
	return r;
}

STrackedObject CTransformation::transform3D(STrackedObject o) {
	STrackedObject result;
	o.x = o.x - orig3D.x;
	o.y = o.y - orig3D.y;
	o.z = o.z - orig3D.z;
	result.x = to3D[0][0] * o.x + to3D[0][1] * o.y + to3D[0][2] * o.z;
	result.y = to3D[1][0] * o.x + to3D[1][1] * o.y + to3D[1][2] * o.z;
	result.z = to3D[2][0] * o.x + to3D[2][1] * o.y + to3D[2][2] * o.z;
	result.error = establishError(result);
	return result;
}

void CTransformation::loadCalibration(const char *str) {
	FILE* file = fopen(str, "r+");
	if (file == NULL) {
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++)
				to3D[i][j] = 0;
		}
		orig3D.x = orig3D.y = orig3D.z = 0;
		for (int i = 0; i < 9; i++)
			hom[i] = 0;
		hom[8] = 1;
	} else {
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				fscanf(file, "%f ", &to3D[i][j]);
			}
			fscanf(file, "\n");
		}
		fscanf(file, "%f %f %f\n", &orig3D.x, &orig3D.y, &orig3D.z);
		for (int i = 0; i < 9; i++) {
			fscanf(file, "%f ", &hom[i]);
			if (i % 3 == 2)
				fscanf(file, "\n");
		}
		fclose(file);
	}
}

void CTransformation::saveCalibration(const char *str) {
	FILE* file = fopen(str, "w+");
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			fprintf(file, "%f ", to3D[i][j]);
		}
		fprintf(file, "\n");
	}
	fprintf(file, "%f %f %f\n", orig3D.x, orig3D.y, orig3D.z);
	for (int i = 0; i < 9; i++) {
		fprintf(file, "%f ", hom[i]);
		if (i % 3 == 2)
			fprintf(file, "\n");
	}
	fclose(file);
}

//this function if meant for debugging
float CTransformation::establishError(STrackedObject o) {
	STrackedObject result;
	float scale = 0.625;
	result.x = (o.x / scale - rintf(o.x / scale)) * scale;
	result.y = (o.y / scale - rintf(o.y / scale)) * scale;
	result.z = (o.z / scale - rintf(o.z / scale)) * scale;
	return sqrt(result.x * result.x + result.y * result.y + result.z * result.z);
}

STrackedObject CTransformation::normalize(STrackedObject o) {
	float scale = sqrt(o.x * o.x + o.y * o.y + o.z * o.z);
	STrackedObject r;
	r.x = o.x / scale;
	r.y = o.y / scale;
	r.z = o.z / scale;
	return r;
}

int CTransformation::calibrate2D(STrackedObject *inp, float dimX, float dimY) {
	STrackedObject r[4];
	STrackedObject o[4];
	o[0].x = inp[1].x - inp[0].x;
	o[0].y = inp[1].y - inp[0].y;
	o[0].z = inp[1].z - inp[0].z;
	o[1].x = inp[2].x - inp[0].x;
	o[1].y = inp[2].y - inp[0].y;
	o[1].z = inp[2].z - inp[0].z;
	//float dimX = //rintf(sqrt(o[0].x*o[0].x+o[0].y*o[0].y+o[0].z*o[0].z)/gridDim)*gridDim;
	//float dimY = //rintf(sqrt(o[1].x*o[1].x+o[1].y*o[1].y+o[1].z*o[1].z)/gridDim)*gridDim;
	r[0].x = 0;
	r[0].y = 0;
	r[1].x = dimX;
	r[1].y = 0;
	r[2].x = 0;
	r[2].y = dimY;
	r[3].x = dimX;
	r[3].y = dimY;
	int sign = 0;
	gsl_matrix *est = gsl_matrix_alloc(8, 8);
	gsl_vector *v = gsl_vector_alloc(8);
	gsl_vector *a = gsl_vector_alloc(8);
	gsl_permutation *dmy = gsl_permutation_alloc(8);
	for (int i = 0; i < 4; i++) {
		o[i].x = -inp[i].y / inp[i].x;
		o[i].y = -inp[i].z / inp[i].x;
	}
	for (int i = 0; i < 4; i++) {
		gsl_matrix_set(est, 2 * i, 0, -o[i].x);
		gsl_matrix_set(est, 2 * i, 1, -o[i].y);
		gsl_matrix_set(est, 2 * i, 2, -1);
		gsl_matrix_set(est, 2 * i, 3, 0);
		gsl_matrix_set(est, 2 * i, 4, 0);
		gsl_matrix_set(est, 2 * i, 5, 0);
		gsl_matrix_set(est, 2 * i, 6, r[i].x * o[i].x);
		gsl_matrix_set(est, 2 * i, 7, r[i].x * o[i].y);
		gsl_matrix_set(est, 2 * i + 1, 0, 0);
		gsl_matrix_set(est, 2 * i + 1, 1, 0);
		gsl_matrix_set(est, 2 * i + 1, 2, 0);
		gsl_matrix_set(est, 2 * i + 1, 3, -o[i].x);
		gsl_matrix_set(est, 2 * i + 1, 4, -o[i].y);
		gsl_matrix_set(est, 2 * i + 1, 5, -1);
		gsl_matrix_set(est, 2 * i + 1, 6, r[i].y * o[i].x);
		gsl_matrix_set(est, 2 * i + 1, 7, r[i].y * o[i].y);
		gsl_vector_set(v, 2 * i, -r[i].x);
		gsl_vector_set(v, 2 * i + 1, -r[i].y);
	}
	gsl_linalg_LU_decomp(est, dmy, &sign);
	gsl_linalg_LU_solve(est, dmy, v, a);
	for (int i = 0; i < 8; i++)
		hom[i] = gsl_vector_get(a, i);
	hom[8] = 1;
	gsl_permutation_free(dmy);
	gsl_matrix_free(est);
	gsl_vector_free(a);
	gsl_vector_free(v);
	transformType = TRANSFORM_2D;
	return 0;
}

int CTransformation::calibrate3D(STrackedObject *o, float gridDimX,
		float gridDimY) {
	STrackedObject v[3];
	int sign = 0;
	gsl_matrix *m23D = gsl_matrix_alloc(3, 3);
	gsl_matrix *inv = gsl_matrix_alloc(3, 3);
	gsl_permutation *dmy = gsl_permutation_alloc(3);
	orig3D = o[0];
	for (int i = 0; i < 2; i++) {
		v[i].x = o[i + 1].x - o[0].x;
		v[i].y = o[i + 1].y - o[0].y;
		v[i].z = o[i + 1].z - o[0].z;
		v[i] = normalize(v[i]);
	}
	//gridDimX = rintf(sqrt(v[0].x*v[0].x+v[0].y*v[0].y+v[0].z*v[0].z)/gridDim);
	//gridDimY = rintf(sqrt(v[1].x*v[1].x+v[1].y*v[1].y+v[1].z*v[1].z)/gridDim);
	/*v[0].x = v[0].x;
	 v[0].y = v[0].y;
	 v[0].z = v[0].z;
	 v[1].x = v[1].x;
	 v[1].y = v[1].y;
	 v[1].z = v[1].z;*/
	v[2].x = +v[0].y * v[1].z - v[1].y * v[0].z;
	v[2].y = +v[0].z * v[1].x - v[1].z * v[0].x;
	v[2].z = +v[0].x * v[1].y - v[1].x * v[0].y;
	//v[2] = normalize(v[2]);
	for (int i = 0; i < 3; i++) {
		//printf("3D calibration matrix: %f %f %f\n",v[i].x,v[i].y,v[i].z);
		v[i] = normalize(v[i]);
		gsl_matrix_set(m23D, 0, i, v[i].x);
		gsl_matrix_set(m23D, 1, i, v[i].y);
		gsl_matrix_set(m23D, 2, i, v[i].z);
	}
	gsl_linalg_LU_decomp(m23D, dmy, &sign);
	gsl_linalg_LU_invert(m23D, dmy, inv);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			to3D[i][j] = gsl_matrix_get(inv, i, j);
		}
	}
	gsl_permutation_free(dmy);
	gsl_matrix_free(m23D);
	gsl_matrix_free(inv);
	transformType = TRANSFORM_3D;
	return 0;
}

//implemented according to   
STrackedObject CTransformation::eigen(double data[]) {
	STrackedObject result;
	result.error = 0;
	gsl_matrix_view m = gsl_matrix_view_array(data, 3, 3);
	gsl_vector *eval = gsl_vector_alloc(3);
	gsl_matrix *evec = gsl_matrix_alloc(3, 3);
	//
	gsl_eigen_symmv_workspace * w = gsl_eigen_symmv_alloc(3);
	gsl_eigen_symmv(&m.matrix, eval, evec, w);
	gsl_eigen_symmv_free(w);
	gsl_eigen_symmv_sort(eval, evec, GSL_EIGEN_SORT_ABS_ASC);

	//eigenvalues
	float L1 = gsl_vector_get(eval, 1);
	float L2 = gsl_vector_get(eval, 2);
	float L3 = gsl_vector_get(eval, 0);
	//eigenvectors
	int V2 = 2;
	int V3 = 0;

	//detected pattern position
	float z = trackedObjectDiameter / sqrt(-L2 * L3) / 2.0;
	float c0 = sqrt((L2 - L1) / (L2 - L3));
	float c0x = c0 * gsl_matrix_get(evec, 0, V2);
	float c0y = c0 * gsl_matrix_get(evec, 1, V2);
	float c0z = c0 * gsl_matrix_get(evec, 2, V2);
	float c1 = sqrt((L1 - L3) / (L2 - L3));
	float c1x = c1 * gsl_matrix_get(evec, 0, V3);
	float c1y = c1 * gsl_matrix_get(evec, 1, V3);
	float c1z = c1 * gsl_matrix_get(evec, 2, V3);

	float z0 = -L3 * c0x + L2 * c1x;
	float z1 = -L3 * c0y + L2 * c1y;
	float z2 = -L3 * c0z + L2 * c1z;
	float s1, s2;
	s1 = s2 = 1;
	float n0 = +s1 * c0x + s2 * c1x;
	float n1 = +s1 * c0y + s2 * c1y;
	float n2 = +s1 * c0z + s2 * c1z;

	//n0 = -L3*c0x-L2*c1x;
	//n1 = -L3*c0y-L2*c1y;
	//n2 = -L3*c0z-L2*c1z;

	//rotate the vector accordingly
	if (z2 * z < 0) {
		z2 = -z2;
		z1 = -z1;
		z0 = -z0;
		//	 n0 = -n0;
		//	 n1 = -n1;
		//	 n2 = -n2;
	}
	result.x = z2 * z;
	result.y = -z0 * z;
	result.z = -z1 * z;
	result.pitch = n0;	//cos(segment.m1/segment.m0)/M_PI*180.0;
	result.roll = n1;	//atan2(segment.v1,segment.v0)/M_PI*180.0;
	result.yaw = n2;	//segment.v1/segment.v0;
	//result.roll = n2*z;	
	//result.pitch = -n0*z;	
	//result.yaw = -n1*z;

	gsl_vector_free(eval);
	gsl_matrix_free(evec);

	return result;
}

STrackedObject CTransformation::transform(SSegment segment, bool unbarreli) {
	float x, y, x1, x2, y1, y2, major, minor, v0, v1;
	STrackedObject result;
	fullUnbarrel = unbarreli;

	//Transform to the Canonical camera coordinates
	x = segment.x;
	y = segment.y;
	transformXY(&x, &y);
	//major axis
	//vertices in image coords
	x1 = segment.x + segment.v0 * segment.m0 * 2;
	x2 = segment.x - segment.v0 * segment.m0 * 2;
	y1 = segment.y + segment.v1 * segment.m0 * 2;
	y2 = segment.y - segment.v1 * segment.m0 * 2;
	//vertices in canonical camera coords 
	transformXY(&x1, &y1);
	transformXY(&x2, &y2);
	//semiaxes length 
	major = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) / 2.0;
	v0 = (x2 - x1) / major / 2.0;
	v1 = (y2 - y1) / major / 2.0;

	//the minor axis 
	//vertices in image coords
	x1 = segment.x + segment.v1 * segment.m1 * 2;
	x2 = segment.x - segment.v1 * segment.m1 * 2;
	y1 = segment.y - segment.v0 * segment.m1 * 2;
	y2 = segment.y + segment.v0 * segment.m1 * 2;
	//vertices in canonical camera coords 
	transformXY(&x1, &y1);
	transformXY(&x2, &y2);
	//minor axis length 
	minor = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) / 2.0;

	//Construct the ellipse characteristic equation
	double a, b, c, d, e, f;
	a = v0 * v0 / (major * major) + v1 * v1 / (minor * minor);
	b = v0 * v1 * (1.0 / (major * major) - 1.0 / (minor * minor));
	c = v0 * v0 / (minor * minor) + v1 * v1 / (major * major);
	d = (-x * a - b * y);
	e = (-y * c - b * x);
	f = (a * x * x + c * y * y + 2 * b * x * y - 1.0);

	//transformation to global coordinates
	double data[] = { a, b, d, b, c, e, d, e, f };
	//homographic transform
	if (transformType == TRANSFORM_2D) {
		result.x = x;
		result.y = y;
		result = transform2D(result);
	}
	//transform
	if (transformType == TRANSFORM_NONE) {
		result = eigen(data);
	}
	if (transformType == TRANSFORM_3D) {
		result = eigen(data);
		float d = sqrt(
				result.x * result.x + result.y * result.y
						+ result.z * result.z);
		result = transform3D(result);
		result.d = d;
	}
	result.pitch = minor < major ? acos(minor / major) / M_PI * 180.0 : 0; //TODO
	result.roll = segment.horizontal; //TODO
	result.yaw = segment.angle / M_PI * 180.0; //TODO
	return result;
}

