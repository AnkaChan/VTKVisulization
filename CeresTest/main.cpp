#define WORKER 2

#if WORKER == 1
// out-dated
#include "triangulation_chb.h"
#elif WORKER == 2

#include "triangulation_allposes.h"
#elif WORKER == 3
// out-dated
#include "bundle_adjustment_single.h"
#elif WORKER == 4
// out-dated
#include "bundle_adjustment_multiple.h"
#elif WORKER == 5

#include "bundle_adjustment_multiple_cycled.h"
#elif WORKER == 6

#include "bundle_adjustment_chb6dof.h"
#endif

int main() {
	Worker *worker;
	
#if WORKER == 1
	worker = new TriangulationChb();
#elif WORKER == 2
	worker = new TriangulationAllPoses();
#elif WORKER == 3
	worker = new BundleAdjustmentSingle();
#elif WORKER == 4
	worker = new BundleAdjustmentMultiple();
#elif WORKER == 5
	worker = new BundleAdjustmentMultipleCycled();
#elif WORKER == 6
	worker = new BundleAdjustmentChb6Dof();
#endif	

	int result = worker->DoWork();

	delete worker;
	return 0;
}