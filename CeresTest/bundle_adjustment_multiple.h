#pragma once
#include "worker.h"

class BundleAdjustmentMultiple : public Worker {
public:
	BundleAdjustmentMultiple();
	int DoWork();
};
