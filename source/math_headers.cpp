#include "math_headers.h"

glm::vec3 Eigen2GLM(const EigenVector3& eigen_vector)
{
    return glm::vec3(eigen_vector[0], eigen_vector[1], eigen_vector[2]);
}
EigenVector3 GLM2Eigen(const glm::vec3& glm_vector)
{
    return EigenVector3(glm_vector[0], glm_vector[1], glm_vector[2]);
}

double Height(const EigenVector3& v1, const EigenVector3& v2, const EigenVector3& v3, const EigenVector3& v4)
{
	// compute normal vector
	double dtmp_x = (v2.y() - v1.y())*(v3.z() - v1.z()) - (v2.z() - v1.z())*(v3.y() - v1.y());
	double dtmp_y = (v2.z() - v1.z())*(v3.x() - v1.x()) - (v2.x() - v1.x())*(v3.z() - v1.z());
	double dtmp_z = (v2.x() - v1.x())*(v3.y() - v1.y()) - (v2.y() - v1.y())*(v3.x() - v1.x());

	// normalize normal vector
	const double dtmp1 = 1.0 / sqrt(dtmp_x*dtmp_x + dtmp_y*dtmp_y + dtmp_z*dtmp_z);
	dtmp_x *= dtmp1;
	dtmp_y *= dtmp1;
	dtmp_z *= dtmp1;

	return (v4.x() - v1.x())*dtmp_x + (v4.y() - v1.y())*dtmp_y + (v4.z() - v1.z())*dtmp_z;
}