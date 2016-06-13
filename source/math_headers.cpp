#include "math_headers.h"

glm::vec3 Eigen2GLM(const EigenVector3& eigen_vector)
{
    return glm::vec3(eigen_vector[0], eigen_vector[1], eigen_vector[2]);
}
EigenVector3 GLM2Eigen(const glm::vec3& glm_vector)
{
    return EigenVector3(glm_vector[0], glm_vector[1], glm_vector[2]);
}