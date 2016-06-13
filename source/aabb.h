#pragma once

#include "iostream"
#include "math_headers.h"

// class "Axis Aligned Bounding Box"
class AABB
{
	public:
		AABB()
		{
			x_min_ = 0.0;	x_max_ = 0.0;
			y_min_ = 0.0;	y_max_ = 0.0;
			z_min_ = 0.0;	z_max_ = 0.0;
			isnt_empty_ = false;
		}

		AABB(double x_min, double y_min, double z_min, double x_max, double y_max, double z_max):
			x_min_(x_min),
			y_min_(y_min),
			z_min_(z_min),
			x_max_(x_max),
			y_max_(y_max),
			z_max_(z_max)
		{
			assert(x_min_ <= x_max_);
			assert(y_min_ <= y_max_);
			assert(z_min_ <= z_max_);
			isnt_empty_ = true;
		}


		~AABB(){}

		bool AddVertex(double x, double y, double z, double margin);

		double x_min_, x_max_;
		double y_min_, y_max_;
		double z_min_, z_max_;
		bool isnt_empty_;

	private:
		DISALLOW_COPY_AND_ASSIGN(AABB);
};