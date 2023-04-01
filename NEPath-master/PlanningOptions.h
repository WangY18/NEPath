#pragma once

struct DirectParallelOptions {
	double delta = 1.5;
	double angle = 0.0;
};

struct ContourParallelOptions {
	double delta = 1.5;
	bool wash = true;
	double washdis = 0.2;
	int num_least = 50;
};