#pragma once

#include "iostream"
#include "math_headers.h"
#include "stack"

#include "aabb.h"

// class "Node of Bounding Volume Hierarchy"
class NodeBVH
{
	public:
		NodeBVH() { }
		~NodeBVH(){ }

		int index_root_;		// index of root node
		int index_child_[2];	// index of child nodes. if index_child1==-1, this is leaf node

		int index_hierarchy_;// index of hierarchy

	private:
		DISALLOW_COPY_AND_ASSIGN(NodeBVH);
};

int MakeBVHTopologyTopDown(const EigenVectorXs& vec_vertex, const std::vector<int>& triangles, std::vector<NodeBVH>& node_bvhs);

static void MakeSurroundingTriangles(std::vector<int>& surrounding_triangles,
									 //////////
									 const int vertex_number, 
								     const std::vector<int>& triangles);

void DevideTriangles(int index_root, 
					 std::vector<int>& triangle2node, 
					 std::vector<NodeBVH>& node_bvhs,
					 ///////////
					 const EigenVectorXs& vec_vertex, 
	                 const std::vector<int>& triangle_list, 
	                 const std::vector<int>& triangles, 
	                 const std::vector<int>& surrounding_triangles);


// add vertex to AABB Bounding Box
void AddVertex(AABB& bounding_box_aabb, const EigenVector3& vec_new_vertex, double margin);

// compute Gravity Center of "index_triangle"-th triangle 
EigenVector3 ComputeTriangleGravityCenter(int index_triangle, const EigenVectorXs& vec_vertex, const std::vector<int>& triangles);