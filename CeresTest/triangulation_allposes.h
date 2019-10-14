#pragma once
#define  _CRT_SECURE_NO_WARNINGS
#include "worker.h"

class TriangulationAllPoses: public Worker {
public:
	TriangulationAllPoses();
	int DoWork();
};