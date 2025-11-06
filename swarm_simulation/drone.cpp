#include "drone.hpp"

drone::drone(Point loc)
{
	this->currLocation = loc;
	this->toScan = true;
}
