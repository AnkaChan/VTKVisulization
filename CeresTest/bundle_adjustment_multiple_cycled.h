#pragma once
#include "worker.h"

class BundleAdjustmentMultipleCycled : public Worker {
public:
	BundleAdjustmentMultipleCycled();
	int DoWork();
private:
	double ComputeReprojectionError(double* cam_param, double* world_point, double measured_x, double measured_y);
};
