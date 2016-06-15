#ifndef _BVH_H_
#define _BVH_H_

#include "iostream"
#include "math_headers.h"
#include "stack"

#include "aabb.h"

// class "Node of Bounding Volume Hierarchy"
class NodeBVH
{
	public:
		NodeBVH() { }
		NodeBVH(const NodeBVH& node_bvh)
			:index_root_(node_bvh.index_root_),
			index_hierarchy_(node_bvh.index_hierarchy_)
		{
			index_child_[0] = node_bvh.index_child_[0];
			index_child_[1] = node_bvh.index_child_[1];
		}
		~NodeBVH(){ }

		int index_root_;		// index of root node
		int index_child_[2];	// index of child nodes. if index_child1==-1, this is leaf node

		int index_hierarchy_;// index of hierarchy

	private:
		//DISALLOW_COPY_AND_ASSIGN(NodeBVH);
};

void BuildBoundingBox
	(int index_bvh,
	double delta,
	std::vector<AABB>& aabb_bvhs,
	///////////////
	const EigenVectorXs& vec_vertex,
	const std::vector<int>& triangles,
	const std::vector<NodeBVH>& node_bvhs);


int MakeBVHTopologyTopDown
	(std::vector<NodeBVH>& node_bvhs,
	///////////////
	const EigenVectorXs& vec_vertex, 
	const std::vector<int>& triangles);


static void MakeSurroundingTriangles
	(std::vector<int>& surrounding_triangles,
	//////////
	const int vertex_number, 
	const std::vector<int>& triangles);


void DevideTriangles
	(int index_root, 
	std::vector<int>& triangle2node, 
	std::vector<NodeBVH>& node_bvhs,
	///////////////
	const EigenVectorXs& vec_vertex, 
	const std::vector<int>& triangle_list, 
	const std::vector<int>& triangles, 
	const std::vector<int>& surrounding_triangles);


// add vertex to AABB Bounding Box
void AddVertex(AABB& bounding_box_aabb, const EigenVector3& vec_new_vertex, double margin);

// compute Gravity Center of "index_triangle"-th triangle 
EigenVector3 ComputeTriangleGravityCenter(int index_triangle, const EigenVectorXs& vec_vertex, const std::vector<int>& triangles);

// compute distance between triangle and vertex
double ComputeDistanceFaceVertex
	(double& w0,
	double& w1,
	double& w2,
	///////////////
	const EigenVector3& vec_triangle_vertex0,
	const EigenVector3& vec_triangle_vertex1,
	const EigenVector3& vec_triangle_vertex2,
	const EigenVector3& vec_vertex
	);

double ComputeDistanceFaceVertex
	(double& w0,
	double& w1,
	double& w2,
	EigenVector3& vec_direction,
	///////////////
	const EigenVector3& vec_triangle_vertex0,
	const EigenVector3& vec_triangle_vertex1,
	const EigenVector3& vec_triangle_vertex2,
	const EigenVector3& vec_vertex
	);

bool IsContactFaceVertex
	(int index_triangle_vertex0,
	int index_triangle_vertex1,
	int index_triangle_vertex2,
	int index_vertex,
	const EigenVector3& vec_triangle_vertex0,
	const EigenVector3& vec_triangle_vertex1,
	const EigenVector3& vec_triangle_vertex2,
	const EigenVector3& vec_vertex,
	const AABB& aabb,
	const double delta);

// for collison between cloth and body
bool IsContactFaceVertex
	(double& distance,
	EigenVector3& vec_direction,
	///////////////
	const EigenVector3& vec_triangle_vertex0,
	const EigenVector3& vec_triangle_vertex1,
	const EigenVector3& vec_triangle_vertex2,
	const EigenVector3& vec_vertex,
	const AABB& aabb,
	const double delta);

bool HasCollision
	(double& distance,
	EigenVector3& vec_direction,
	///////////////
	int index_bvh,
	const EigenVector3& vec_vertex,
	const EigenVectorXs& vec_vertices,
	const std::vector<int>& triangles,
	const std::vector<NodeBVH>& node_bvhs,
	const std::vector<AABB>& aabb_bvhs);

// class "Contact Element"
class ContactElement
{
	public:
		ContactElement(bool is_face_vertex, int j0, int j1, int j2, int j3)
		{
			this->is_face_vertex_ = is_face_vertex;
			if (is_face_vertex) {
				this->is_face_vertex_ = true;
				// reordering for fast search
				if (j0 < j1 && j0 < j2 && j1 < j2) { index0_ = j0;  index1_ = j1;  index2_ = j2;  index3_ = j3; }
				else if (j0 < j1 && j0 < j2 && j2 < j1) { index0_ = j0;  index1_ = j2;  index2_ = j1;  index3_ = j3; }
				else if (j1 < j0 && j1 < j2 && j0 < j2) { index0_ = j1;  index1_ = j0;  index2_ = j2;  index3_ = j3; }
				else if (j1 < j0 && j1 < j2 && j2 < j0) { index0_ = j1;  index1_ = j2;  index2_ = j0;  index3_ = j3; }
				else if (j2 < j0 && j2 < j1 && j0 < j1) { index0_ = j2;  index1_ = j0;  index2_ = j1;  index3_ = j3; }
				else if (j2 < j0 && j2 < j1 && j1 < j0) { index0_ = j2;  index1_ = j1;  index2_ = j0;  index3_ = j3; }
				else { assert(0); }
			}
			else {
				this->is_face_vertex_ = false;
				// reordering for fast search      
				if (j0 < j1 && j0 < j2 && j0 < j3 && j2 < j3) { index0_ = j0;  index1_ = j1;  index2_ = j2;  index3_ = j3; }
				else if (j0 < j1 && j0 < j2 && j0 < j3 && j3 < j2) { index0_ = j0;  index1_ = j1;  index2_ = j3;  index3_ = j2; }
				else if (j1 < j0 && j1 < j2 && j1 < j3 && j2 < j3) { index0_ = j1;  index1_ = j0;  index2_ = j2;  index3_ = j3; }
				else if (j1 < j0 && j1 < j2 && j1 < j3 && j3 < j2) { index0_ = j1;  index1_ = j0;  index2_ = j3;  index3_ = j2; }
				else if (j2 < j0 && j2 < j1 && j2 < j3 && j0 < j1) { index0_ = j2;  index1_ = j3;  index2_ = j0;  index3_ = j1; }
				else if (j2 < j0 && j2 < j1 && j2 < j3 && j1 < j0) { index0_ = j2;  index1_ = j3;  index2_ = j1;  index3_ = j0; }
				else if (j3 < j0 && j3 < j1 && j3 < j2 && j0 < j1) { index0_ = j3;  index1_ = j2;  index2_ = j0;  index3_ = j1; }
				else if (j3 < j0 && j3 < j1 && j3 < j2 && j1 < j0) { index0_ = j3;  index1_ = j2;  index2_ = j1;  index3_ = j0; }
				else { assert(0); }
			}
		}

		bool operator < (const ContactElement& contact_element) const
		{
			if (index0_ != contact_element.index0_) { return index0_ < contact_element.index0_; }
			if (index1_ != contact_element.index1_) { return index1_ < contact_element.index1_; }
			if (index2_ != contact_element.index2_) { return index2_ < contact_element.index2_; }
			return index3_ < contact_element.index3_;
		}


		bool is_face_vertex_;
		int index0_, index1_, index2_, index3_;
};

#endif