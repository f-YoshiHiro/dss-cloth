#ifndef _IO_MESH_H_
#define _IO_MESH_H_

#include "math_headers.h"
//#include <vector>
#include <iostream>
#include <string>
#include <igl/read_triangle_mesh.h>

typedef enum MeshDimension {
	kMeshDimension2D,
	kMeshDimension3D,
	kMeshDimensionTotalNumber
};

typedef enum FileType {
	kFileTypeObj,
	kFileTypeMesh,
	kFileTypeTotalNumber
};

class IGLMeshLoader
{
public:
	IGLMeshLoader();
	IGLMeshLoader(char* _file_path, MeshDimension _dim, FileType _file_type, float _scale = 10.0f, EigenVector3 _translate = EigenVector3(0.0f, 4.0f, 0.0f));

	inline bool is_loaded() { return is_loaded_; }

	// accesser / mutater
	uint vertex_number() { return mat_vertex_.rows(); }
	uint triangle_number() { return mat_face_.rows(); }
	uint tetrahedron_number() { return mat_tetrahedron_.rows(); }

	EigenVector3 vertex(uint i) { return mat_vertex_.row(i).transpose(); }
	uint triangle(uint itri, uint ivtx) { return mat_face_(itri, ivtx); }
	uint tetrahedron(uint itet, uint ivtx) { return mat_tetrahedron_(itet, ivtx); }

private:
	bool is_loaded_;

	MeshDimension mesh_dimension_;
	FileType file_type_;

	EigenMatrixXs mat_vertex_;	    // vertices
	EigenMatrixXi mat_face_;        // tri index
	EigenMatrixXi mat_tetrahedron_;  // tet index
};

#endif