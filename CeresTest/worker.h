#pragma once
#define  _CRT_SECURE_NO_WARNINGS
#include <cmath>
#include <cstdio>
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
using std::cout;
using std::endl;

#define LEN_IMAGE_NAME 5
#define NUM_CAMERA 8
// camera properties
struct CameraParameters {
	int index;
	double ax[3];
	double trans[3];
	double fx, fy;
	double cx, cy;
	double k1, k2, k3;
	double p1, p2;

	CameraParameters(int index_, double ax1_, double ax2_, double ax3_, double t1_, double t2_, double t3_, double fx_, double fy_, double cx_, double cy_, double k1_, double k2_, double p1_, double p2_, double k3_)
		:index(index_), ax{ ax1_, ax2_, ax3_ }, trans{ t1_, t2_, t3_ }, fx(fx_), fy(fy_), cx(cx_), cy(cy_), k1(k1_), k2(k2_), k3(k3_), p1(p1_), p2(p2_) {
		data_array = new double[15];
		data_array[0] = ax1_;
		data_array[1] = ax2_;
		data_array[2] = ax3_;
		data_array[3] = t1_;
		data_array[4] = t2_;
		data_array[5] = t3_;
		data_array[6] = fx_;
		data_array[7] = fy_;
		data_array[8] = cx_;
		data_array[9] = cy_;
		data_array[10] = k1_;
		data_array[11] = k2_;
		data_array[12] = p1_;
		data_array[13] = p2_;
		data_array[14] = k3_;
	}
	CameraParameters() {}
	~CameraParameters() {
		delete[] data_array;
	}
	double* GetDataArray() {
		return data_array;
	}

	void Print() {
		printf("Camera[%d] -------\nax: (%.4f, %.4f, %.4f), t: (%.4f, %.4f, %.4f),\nf: (%.4f, %.4f), c: (%.4f, %.4f), k: (%.4f, %.4f, %.4f), p: (%.4f, %.4f)\n\n",
			index,
			ax[0], ax[1], ax[2],
			trans[0], trans[1], trans[2],
			fx, fy,
			cx, cy,
			k1, k2, k3,
			p1, p2);
	}
private:
	double *data_array;
};

class Worker {
public:
	Worker();
	virtual int DoWork();
	int LoadCameraParameters(const char* xml_path, CameraParameters* (&cam_params)[NUM_CAMERA]);
	int LoadMeasurementExclusion(const char* txt_path, bool* (&excluded_out));
	};
