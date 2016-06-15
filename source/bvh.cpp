#include "bvh.h"

void BuildBoundingBox
	(int index_bvh,
	double delta,
	std::vector<AABB>& aabb_bvhs,
	//////////
	const EigenVectorXs& vec_vertex,
	const std::vector<int>& triangles,
	const std::vector<NodeBVH>& node_bvhs)
{
	aabb_bvhs.resize(node_bvhs.size());
	assert(index_bvh < node_bvhs.size());

	int index_child0 = node_bvhs[index_bvh].index_child_[0];
	int index_child1 = node_bvhs[index_bvh].index_child_[1];

	if (index_child1 == -1)	// leaf node 
	{
		int index_triangle = index_child0;
		assert(index_triangle < triangles.size());

		int index_vertex0 = triangles[index_triangle * 3 + 0];
		int index_vertex1 = triangles[index_triangle * 3 + 1];
		int index_vertex2 = triangles[index_triangle * 3 + 2];

		const EigenVector3 vec_vertex0 = vec_vertex.block_vector(index_vertex0);
		const EigenVector3 vec_vertex1 = vec_vertex.block_vector(index_vertex1);
		const EigenVector3 vec_vertex2 = vec_vertex.block_vector(index_vertex2);

		AABB& bounding_box = aabb_bvhs[index_bvh];
		bounding_box.isnt_empty_ = false;

		bounding_box.AddVertex(vec_vertex0.x(), vec_vertex0.y(), vec_vertex0.z(), delta*0.5);
		bounding_box.AddVertex(vec_vertex1.x(), vec_vertex1.y(), vec_vertex1.z(), delta*0.5);
		bounding_box.AddVertex(vec_vertex2.x(), vec_vertex2.y(), vec_vertex2.z(), delta*0.5);

		return;
	}

	// internal node
	assert(node_bvhs[index_child0].index_root_ == index_bvh);
	assert(node_bvhs[index_child1].index_root_ == index_bvh);
	
	BuildBoundingBox(index_child0, delta, aabb_bvhs, vec_vertex, triangles, node_bvhs);
	BuildBoundingBox(index_child1, delta, aabb_bvhs, vec_vertex, triangles, node_bvhs);

	AABB& bounding_box = aabb_bvhs[index_bvh];
	bounding_box.isnt_empty_ = false;
	bounding_box  = aabb_bvhs[index_child0];
	bounding_box += aabb_bvhs[index_child1];

	return;
}

int MakeBVHTopologyTopDown
	(std::vector<NodeBVH>& node_bvhs,
	////////////////
	const EigenVectorXs& vec_vertex, 
	const std::vector<int>& triangles)
{
	std::cout << "Make BVH Topology Top Down" << std::endl;

	std::vector<int> surrounding_triangles;
	MakeSurroundingTriangles(surrounding_triangles, (int)(vec_vertex.size()/3), triangles);

	////////////////////
	// check
	//for (int i = 0; i < surrounding_triangles.size(); ++i)
	//{
	//	std::cout << "surrounding_triangles[" << i << "]:" << surrounding_triangles[i] << std::endl;
	//}
	////////////////////

	node_bvhs.clear();

	const int triangle_number = (int)(triangles.size() / 3);
	std::vector<int> triangle_list(triangle_number);
	for (int i_triangle = 0; i_triangle < triangle_number; ++i_triangle) { triangle_list[i_triangle] = i_triangle; }

	std::vector<int> triangle2node;
	triangle2node.resize(triangle_number, 0);
	node_bvhs.resize(1);
	node_bvhs[0].index_root_ = -1;
	
	DevideTriangles(0, triangle2node, node_bvhs,
		vec_vertex, triangle_list, triangles, surrounding_triangles);

	return 0;
}

static void MakeSurroundingTriangles
	(std::vector<int>& surrounding_triangles,
	////////////////
	const int vertex_number, 
	const std::vector<int>& triangles)
{
	const int number_triangle = (int)(triangles.size()/3);
	const int edge2node[3][2] = { {1,2},{2,0},{0,1} };
	surrounding_triangles.clear();
	surrounding_triangles.resize(number_triangle*3,-1);

	CJaggedArray jarray_tsv;
	jarray_tsv.SetNodeToElem(triangles, number_triangle, 3, vertex_number);
	
	for (int i_triangle = 0; i_triangle < number_triangle; ++i_triangle)
	{
		for (int i_edge = 0; i_edge < 3; ++i_edge)
		{
			int i_vertex0 = triangles[i_triangle * 3 + edge2node[i_edge][0]];
			int i_vertex1 = triangles[i_triangle * 3 + edge2node[i_edge][1]];

			for (int i_sur_triangle = jarray_tsv.index[i_vertex0]; i_sur_triangle < jarray_tsv.index[i_vertex0+1]; ++i_sur_triangle)
			{
				int j_triangle = jarray_tsv.array[i_sur_triangle];
				if (j_triangle == i_triangle) continue;

				for (int j_edge = 0; j_edge < 3; ++j_edge)
				{
					int j_vertex0 = triangles[j_triangle * 3 + edge2node[j_edge][0]];
					int j_vertex1 = triangles[j_triangle * 3 + edge2node[j_edge][1]];
					if (i_vertex0 == j_vertex1 && i_vertex1 == j_vertex0)
					{
						surrounding_triangles[i_triangle * 3 + i_edge] = j_triangle;
						break;
					}
				}

				if (surrounding_triangles[i_triangle * 3 + i_edge] == j_triangle) break;

			}
		}
	}
}

void DevideTriangles(int index_root,
	std::vector<int>& triangle2node,
	std::vector<NodeBVH>& node_bvhs,
	///////////
	const EigenVectorXs& vec_vertex,
	const std::vector<int>& triangle_list,
	const std::vector<int>& triangles,
	const std::vector<int>& surrounding_triangles)
{
	assert(triangle_list.size() > 1);
	double margin = 1.0e-10;

	// compute Gravity Center of each triangles included "triangle_list"
	AABB bounding_box_aabb;
	for (int i_list = 0; i_list < triangle_list.size(); ++i_list)
	{
		int index_triangle = triangle_list[i_list];
		assert(index_triangle < triangles.size() / 3);
		assert(triangle2node[index_triangle] == index_root);
		EigenVector3 vec_gravity_center = ComputeTriangleGravityCenter(i_list, vec_vertex, triangles);
		AddVertex(bounding_box_aabb, vec_gravity_center, margin);
	}

	// decide Devide Direction
	double length_x = bounding_box_aabb.x_max_ - bounding_box_aabb.x_min_;
	double length_y = bounding_box_aabb.y_max_ - bounding_box_aabb.y_min_;
	double length_z = bounding_box_aabb.z_max_ - bounding_box_aabb.z_min_;
	EigenVector3 vec_devide_direction;
	if (length_x > length_y && length_x > length_z) { vec_devide_direction = EigenVector3(1.0, 0.0, 0.0); }
	if (length_y > length_z && length_y > length_x) { vec_devide_direction = EigenVector3(0.0, 1.0, 0.0); }
	if (length_z > length_x && length_z > length_y) { vec_devide_direction = EigenVector3(0.0, 0.0, 1.0); }
	
	// decide Kernel Triangle
	EigenVector3 vec_bounding_box_center((bounding_box_aabb.x_min_+bounding_box_aabb.x_max_)*0.5, (bounding_box_aabb.y_min_ + bounding_box_aabb.y_max_)*0.5, (bounding_box_aabb.z_min_ + bounding_box_aabb.z_max_)*0.5);
	int index_kernel_triangle = -1; // init by -1
	for (int i_list = 0; i_list < triangle_list.size(); ++i_list)
	{
		int index_triangle0 = triangle_list[i_list];
		const EigenVector3& vec_gravity_center0 = ComputeTriangleGravityCenter(index_triangle0, vec_vertex, triangles);
		if (fabs((vec_gravity_center0-vec_bounding_box_center).dot(vec_devide_direction)) < 1.0e-10) continue;	// very cloth to Division Plane
		if ((vec_gravity_center0 - vec_bounding_box_center).dot(vec_devide_direction) < 0) { vec_devide_direction *= -1; }
		break;
	}

	if(index_kernel_triangle == -1)
	{
		vec_bounding_box_center = EigenVector3(0.0, 0.0, 0.0);
		for (int i_list = 0; i_list < triangle_list.size(); ++i_list)
		{
			int index_triangle0 = triangle_list[i_list];
			const EigenVector3& vec_gravity_center0 = ComputeTriangleGravityCenter(index_triangle0, vec_vertex, triangles);
			vec_bounding_box_center += vec_gravity_center0;
		}
		vec_bounding_box_center /= triangle_list.size();

		double mat[3][3] = { {0,0,0},{0,0,0},{0,0,0} };
		for (int i_list = 0; i_list < triangle_list.size(); ++i_list)
		{
			int index_triangle0 = triangle_list[i_list];
			const EigenVector3& vec_gravity_center0 = ComputeTriangleGravityCenter(index_triangle0, vec_vertex, triangles);
			const EigenVector3& vec_gravity_center = vec_gravity_center0 - vec_bounding_box_center;

			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					mat[i][j] += vec_gravity_center[i] * vec_gravity_center[j];
				}
			}
		}
		vec_devide_direction = EigenVector3(1,1,1);

		for (int i = 0; i < 10; ++i)
		{
			double tmp[3] = {
				mat[0][0] * vec_devide_direction[0] + mat[0][1] * vec_devide_direction[1] + mat[0][2] * vec_devide_direction[2],
				mat[1][0] * vec_devide_direction[0] + mat[1][1] * vec_devide_direction[1] + mat[1][2] * vec_devide_direction[2],
				mat[2][0] * vec_devide_direction[0] + mat[2][1] * vec_devide_direction[1] + mat[2][2] * vec_devide_direction[2],
			};

			double length = sqrt(tmp[0]*tmp[0]+ tmp[1]*tmp[1] + tmp[2]*tmp[2]);
			vec_devide_direction[0] = tmp[0] / length;
			vec_devide_direction[1] = tmp[1] / length;
			vec_devide_direction[2] = tmp[2] / length;
		}

		for (int i_list = 0; i_list < triangle_list.size(); ++i_list)
		{
			int index_triangle0 = triangle_list[i_list];
			const EigenVector3& vec_gravity_center0 = ComputeTriangleGravityCenter(index_triangle0, vec_vertex, triangles);
			if (fabs((vec_gravity_center0 - vec_bounding_box_center).dot(vec_devide_direction)) < 1.0e-10) continue;	// very cloth to Division Plane
			if ((vec_gravity_center0 - vec_bounding_box_center).dot(vec_devide_direction) < 0) { vec_devide_direction *= -1; }
			index_kernel_triangle = index_triangle0;
			break;
		}
	}
	///////////////////
	// check 
	//std::cout << "index_kernel_triangle:" << index_kernel_triangle << std::endl;
	///////////////////


	// devide Triangles
	int index_child0 = (int)node_bvhs.size();
	int index_child1 = (int)node_bvhs.size() + 1;
	node_bvhs.resize(node_bvhs.size() + 2);

	node_bvhs[index_child0].index_root_ = index_root;	// root info
	node_bvhs[index_child1].index_root_ = index_root;	// root info
	node_bvhs[index_root].index_child_[0] = index_child0;	// child info
	node_bvhs[index_root].index_child_[1] = index_child0;	// child info

	std::vector<int> list_child0;
	{
		triangle2node[index_kernel_triangle] = index_child0;
		list_child0.push_back(index_kernel_triangle);

		std::stack<int> stack;
		stack.push(index_kernel_triangle);
		while (!stack.empty())
		{
			int i_triangle0 = stack.top();
			stack.pop();

			for (int i_edge = 0; i_edge < 3; ++i_edge)
			{
				//////////////////
				// check
				//std::cout <<"i_triangle0 * 3 + i_edge:"<< i_triangle0 * 3 + i_edge << std::endl;
				//////////////////

				int j_triangle = surrounding_triangles[i_triangle0 * 3 + i_edge];
				if (j_triangle == -1) continue;
				if (triangle2node[j_triangle] != index_root) continue;
				assert(j_triangle < triangles.size() / 3);

				const EigenVector3 vec_gravity_center1 = ComputeTriangleGravityCenter(j_triangle, vec_vertex, triangles);
				if ((vec_gravity_center1 - vec_bounding_box_center).dot(vec_devide_direction) < 0) continue;
				stack.push(j_triangle);
				triangle2node[j_triangle] = index_child0;
				list_child0.push_back(j_triangle);
			} 
		}

		assert(list_child0.size() > 0);
	}

	std::vector<int> list_child1;
	{
		for (int i_list = 0; i_list < triangle_list.size(); ++i_list)
		{
			int i_triangle = triangle_list[i_list];
			if (triangle2node[i_triangle] == index_child0) continue;
			assert(triangle2node[i_triangle]==index_root);

			triangle2node[i_triangle] = index_child1;
			list_child1.push_back(i_triangle);
		}

		assert(list_child1.size() > 0);
	}

	// re-devide about list_child0
	if (list_child0.size() == 1)
	{
		node_bvhs[index_child0].index_child_[0] = list_child0[0];
		node_bvhs[index_child0].index_child_[1] = -1;
	}
	else
	{
		DevideTriangles(index_child0, triangle2node, node_bvhs, 
						vec_vertex, list_child0, triangles, surrounding_triangles);
	}
	list_child0.clear();

	// re-devide about list_child1
	if (list_child1.size() == 1)
	{
		node_bvhs[index_child1].index_child_[0] = list_child0[0];
		node_bvhs[index_child1].index_child_[1] = -1;
	}
	else
	{
		DevideTriangles(index_child1, triangle2node, node_bvhs,
			vec_vertex, list_child1, triangles, surrounding_triangles);
	}
	list_child1.clear();
}

void AddVertex(AABB& bounding_box_aabb, const EigenVector3& vec_new_vertex, double margin)
{
	bounding_box_aabb.AddVertex(vec_new_vertex.x(), vec_new_vertex.y(), vec_new_vertex.z(), margin);
}

EigenVector3 ComputeTriangleGravityCenter(int index_triangle, const EigenVectorXs& vec_vertex, const std::vector<int>& triangles)
{
	int index_vertex0 = triangles[index_triangle * 3 + 0];
	int index_vertex1 = triangles[index_triangle * 3 + 1];
	int index_vertex2 = triangles[index_triangle * 3 + 2];
	EigenVector3 vec_gravity_center = (vec_vertex.block_vector(index_vertex0) + vec_vertex.block_vector(index_vertex1) + vec_vertex.block_vector(index_vertex2)) / 3.0;

	return vec_gravity_center;
}

double ComputeDistanceFaceVertex
	(double& w0,
	double& w1,
	double& w2,
	///////////////
	const EigenVector3& vec_triangle_vertex0,
	const EigenVector3& vec_triangle_vertex1,
	const EigenVector3& vec_triangle_vertex2,
	const EigenVector3& vec_vertex
	)
{
	EigenVector3 vec_0 = vec_triangle_vertex0 - vec_triangle_vertex2;
	EigenVector3 vec_1 = vec_triangle_vertex1 - vec_triangle_vertex2;

	double t0 = vec_0.dot(vec_0);
	double t1 = vec_1.dot(vec_1);
	double t2 = vec_0.dot(vec_1);
	double t3 = vec_0.dot(vec_vertex - vec_triangle_vertex2);
	double t4 = vec_1.dot(vec_vertex - vec_triangle_vertex2);
	double determinant = t0*t1 - t2*t2;
	double inverse_determinat = 1.0 / determinant;

	w0 = (+t1*t3 - t2*t4) * inverse_determinat;
	w1 = (-t2*t3 - t0*t4) * inverse_determinat;
	w2 = 1 - w0 - w1;

	EigenVector3 vec_foot = w0*vec_triangle_vertex0 + w1*vec_triangle_vertex1 + w2*vec_triangle_vertex2;

	return (vec_foot - vec_vertex).norm();
}

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
	)
{
	EigenVector3 vec_0 = vec_triangle_vertex0 - vec_triangle_vertex2;
	EigenVector3 vec_1 = vec_triangle_vertex1 - vec_triangle_vertex2;

	double t0 = vec_0.dot(vec_0);
	double t1 = vec_1.dot(vec_1);
	double t2 = vec_0.dot(vec_1);
	double t3 = vec_0.dot(vec_vertex - vec_triangle_vertex2);
	double t4 = vec_1.dot(vec_vertex - vec_triangle_vertex2);
	double determinant = t0*t1 - t2*t2;
	double inverse_determinat = 1.0 / determinant;

	w0 = (+t1*t3 - t2*t4) * inverse_determinat;
	w1 = (-t2*t3 - t0*t4) * inverse_determinat;
	w2 = 1 - w0 - w1;

	EigenVector3 vec_foot = w0*vec_triangle_vertex0 + w1*vec_triangle_vertex1 + w2*vec_triangle_vertex2;
	vec_direction = vec_foot.normalized();

	return (vec_foot - vec_vertex).norm();
}

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
	const double delta)
{
	if (!aabb.IsInside(vec_vertex.x(), vec_vertex.y(), vec_vertex.z())) return false;

	double height = fabs(Height(vec_triangle_vertex0, vec_triangle_vertex1, vec_triangle_vertex2, vec_vertex));
	if (height > delta) return false;

	double w0, w1, w2;
	const double distance = ComputeDistanceFaceVertex(w0, w1, w2, vec_triangle_vertex0, vec_triangle_vertex1, vec_triangle_vertex2, vec_vertex);
	if (distance > delta) return false;

	double margin = 0.0;
	if (w0 < -margin || w0 > 1 + margin) return false;
	if (w1 < -margin || w1 > 1 + margin) return false;
	if (w2 < -margin || w2 > 1 + margin) return false;

	return true;
}

bool IsContactFaceVertex
	(double& distance,
	EigenVector3& vec_direction,
	///////////////
	const EigenVector3& vec_triangle_vertex0,
	const EigenVector3& vec_triangle_vertex1,
	const EigenVector3& vec_triangle_vertex2,
	const EigenVector3& vec_vertex,
	const AABB& aabb,
	const double delta)
{
	if (!aabb.IsInside(vec_vertex.x(), vec_vertex.y(), vec_vertex.z())) return false;

	double height = fabs(Height(vec_triangle_vertex0, vec_triangle_vertex1, vec_triangle_vertex2, vec_vertex));
	if (height > delta) return false;

	double w0, w1, w2;
	distance = ComputeDistanceFaceVertex(w0, w1, w2, vec_triangle_vertex0, vec_triangle_vertex1, vec_triangle_vertex2, vec_vertex);
	if (distance > delta) return false;



	double margin = 0.0;
	if (w0 < -margin || w0 > 1 + margin) return false;
	if (w1 < -margin || w1 > 1 + margin) return false;
	if (w2 < -margin || w2 > 1 + margin) return false;

	return true;
}

bool HasCollision
	(double& distance,
	EigenVector3& vec_direction,
	///////////////
	int index_bvh,
	const EigenVector3& vec_vertex,
	const EigenVectorXs& vec_vertices,
	const std::vector<int>& triangles,
	const std::vector<NodeBVH>& node_bvhs,
	const std::vector<AABB>& aabb_bvhs)
{
	// if vec_vertex is not inside of B.B., return false
	if (!aabb_bvhs[index_bvh].IsInside(vec_vertex.x(), vec_vertex.y(), vec_vertex.z())) return false;

	// if this is leaf node, compute distance
	if (node_bvhs[index_bvh].index_child_[0] == -1)
	{
		int index_triangle = node_bvhs[index_bvh].index_child_[1];
		assert(index_triangle >= 0);

		int i_tri_vert0 = triangles[index_triangle * 3 + 0];
		int i_tri_vert1 = triangles[index_triangle * 3 + 1];
		int i_tri_vert2 = triangles[index_triangle * 3 + 2];

		const bool is_contact = IsContactFaceVertex
								(distance, 
								vec_direction, 
								vec_vertices.block_vector(i_tri_vert0), 
								vec_vertices.block_vector(i_tri_vert1), 
								vec_vertices.block_vector(i_tri_vert2),
								vec_vertex,
								aabb_bvhs[index_bvh],
								0.01);

		if (is_contact) {
			return true;
		}
		else {
			return false;
		}
	}

	// if this is internal node, do recursion
	HasCollision
		(distance, vec_direction,
		node_bvhs[index_bvh].index_child_[0], vec_vertex, vec_vertices, triangles, node_bvhs, aabb_bvhs);

	HasCollision
		(distance, vec_direction,
		node_bvhs[index_bvh].index_child_[1], vec_vertex, vec_vertices, triangles, node_bvhs, aabb_bvhs);
}