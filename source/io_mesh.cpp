#include "io_mesh.h"

IGLMeshLoader::IGLMeshLoader():
	is_loaded_(false)
{
	
}

IGLMeshLoader::IGLMeshLoader(char* _file_path, MeshDimension _dim, FileType _file_type, float scale, EigenVector3 translate):
	is_loaded_(false),
	mesh_dimension_(_dim),
	file_type_(_file_type)
{
	std::cout << "iglMeshLoader initializing (to " << _file_path << ").." << std::endl;

	switch (_file_type)
	{
	case kFileTypeObj:
		igl::readOBJ(_file_path, mat_vertex_, mat_face_);
		mat_vertex_ *= scale;
		is_loaded_ = true;
		break;

	case kFileTypeMesh:
		assert(_dim == kMeshDimension3D);
		igl::readMESH(_file_path, mat_vertex_, mat_tetrahedron_, mat_face_);
		mat_vertex_ *= scale;
		is_loaded_ = true;
		break;

	default:
		std::cout << "This type of file is not supportted. (iglMeshLoader::iglMeshLoader)" << std::endl;
		break;
	}
}