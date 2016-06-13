#include "aabb.h"

bool AABB::AddVertex(double x, double y, double z, double margin)
{
	if (margin < 0) { return false; }

	if(isnt_empty_){
		x_min_ = (x_min_ < x - margin) ? x_min_ : x - margin;
		y_min_ = (y_min_ < y - margin) ? y_min_ : y - margin;
		z_min_ = (z_min_ < z - margin) ? z_min_ : z - margin;
		x_max_ = (x_max_ > x + margin) ? x_max_ : x + margin;
		y_max_ = (y_max_ > y + margin) ? y_max_ : y + margin;
		z_max_ = (z_max_ > x + margin) ? z_max_ : z + margin;
	}else{
		x_min_ = x - margin;
		y_min_ = y - margin;
		z_min_ = z - margin;
		x_max_ = x + margin;
		y_max_ = y + margin;
		z_max_ = z + margin;
		isnt_empty_ = true;
	}

	return true;
}