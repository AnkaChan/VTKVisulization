#pragma once
#define  _CRT_SECURE_NO_WARNINGS
#include "worker.h"

class TriangulationChb : public Worker {
public:
	TriangulationChb();
	int DoWork();
};