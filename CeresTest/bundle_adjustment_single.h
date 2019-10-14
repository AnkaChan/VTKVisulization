#pragma once
#include "worker.h"

class BundleAdjustmentSingle : public Worker {
public:
	BundleAdjustmentSingle();
	int DoWork();
};
