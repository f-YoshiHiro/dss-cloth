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

bool AABB::IsInside(double x, double y, double z) const
{
	if (!isnt_empty_) return false;

	if (x >= x_min_ && x <= x_max_
		&& y >= y_min_ && y <= y_max_
		&& z >= z_min_ && z <= z_max_) return true;

	return false;
}

void AABB::Draw(const VBO& vbos)
{
	//Draw axis.
	/*glColor3d(0.8, 0.8, 0.2);
	glBegin(GL_LINES);

	glVertex3d(x_min_, y_max_, z_min_);	glVertex3d(x_max_, y_max_, z_min_);
	glVertex3d(x_max_, y_max_, z_min_); glVertex3d(x_max_, y_max_, z_max_);
	glVertex3d(x_max_, y_max_, z_max_); glVertex3d(x_min_, y_max_, z_max_);
	glVertex3d(x_min_, y_max_, z_max_); glVertex3d(x_min_, y_max_, z_min_);

	glVertex3d(x_min_, y_min_, z_min_);	glVertex3d(x_max_, y_min_, z_min_);
	glVertex3d(x_max_, y_min_, z_min_); glVertex3d(x_max_, y_min_, z_max_);
	glVertex3d(x_max_, y_min_, z_max_); glVertex3d(x_min_, y_min_, z_max_);
	glVertex3d(x_min_, y_min_, z_max_); glVertex3d(x_min_, y_min_, z_min_);
	
	glVertex3d(x_min_, y_max_, z_min_); glVertex3d(x_min_, y_min_, z_min_);
	glVertex3d(x_max_, y_max_, z_min_); glVertex3d(x_max_, y_min_, z_min_);
	glVertex3d(x_max_, y_max_, z_max_); glVertex3d(x_max_, y_min_, z_max_);
	glVertex3d(x_min_, y_max_, z_max_); glVertex3d(x_min_, y_min_, z_max_);
	
	glEnd();*/

	assert(isnt_empty_);

	EigenVector3 vec_center((x_max_-x_min_)/2.0, (y_max_ - y_min_) / 2.0, (z_max_ - z_min_) / 2.0);
	aabb_body_.move_to(Eigen2GLM(vec_center));
	aabb_body_.change_color(glm::vec3(0.5, 0.2, 0.3));
	aabb_body_.Draw(vbos);
}