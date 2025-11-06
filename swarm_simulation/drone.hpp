#pragma once
#include "graph.hpp"
#include "mazeGeneration.hpp"


class drone {
public:
	Point currLocation;
	bool toScan;
	bool isGoingToLocation;

	drone(Point loc);
};
