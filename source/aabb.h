#pragma once

#include "iostream"
#include "openGL_headers.h"
#include "primitive.h"
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

		AABB(const AABB& aabb)
			:x_min_(aabb.x_min_),	x_max_(aabb.x_max_),
			y_min_(aabb.y_min_),	y_max_(aabb.y_max_),
			z_min_(aabb.x_min_),	z_max_(aabb.z_max_),
			isnt_empty_(aabb.isnt_empty_){}

		~AABB(){}

		/*AABB& operator= (const AABB& aabb)
		{
			if (!aabb.isnt_empty_) return *this;

			x_min_ = aabb.x_min_;	x_max_ = aabb.x_max_;
			y_min_ = aabb.y_min_;	y_max_ = aabb.y_max_;
			z_min_ = aabb.z_min_;	z_max_ = aabb.z_max_;
			isnt_empty_ = aabb.isnt_empty_;

			return *this;
		}*/

		AABB& operator+= (const AABB& aabb)
		{
			if (!aabb.isnt_empty_) return *this;

			if (!isnt_empty_)
			{
				x_max_ = aabb.x_max_;	x_min_ = aabb.x_min_;
				y_max_ = aabb.y_max_;	y_min_ = aabb.y_min_;
				z_max_ = aabb.z_max_;	z_min_ = aabb.z_min_;
				this->isnt_empty_ = aabb.isnt_empty_;

				return *this;
			}

			x_max_ = (x_max_ > aabb.x_max_) ? x_max_ : aabb.x_max_;
			x_min_ = (x_min_ < aabb.x_min_) ? x_min_ : aabb.x_min_;
			y_max_ = (y_max_ > aabb.y_max_) ? y_max_ : aabb.y_max_;
			y_min_ = (y_min_ < aabb.y_min_) ? y_min_ : aabb.y_min_;
			z_max_ = (z_max_ > aabb.z_max_) ? z_max_ : aabb.z_max_;
			z_min_ = (z_min_ < aabb.z_min_) ? z_min_ : aabb.z_min_;

			return *this;
		}


		bool AddVertex(double x, double y, double z, double margin);

		bool IsInside(double x, double y, double z) const;


		void Draw(const VBO& vbos);

		double x_min_, x_max_;
		double y_min_, y_max_;
		double z_min_, z_max_;
		bool isnt_empty_;

	private:
		Cube aabb_body_;

		//DISALLOW_COPY_AND_ASSIGN(AABB);
};