#pragma warning( disable : 4267)

#include <fstream>
#include "mesh.h"

//----------State Control--------------------//
extern bool g_enable_bending_constrints;

#pragma region Mesh Class
void Mesh::Reset()
{
    Cleanup();
    Init();
}

void Mesh::Cleanup()
{
    edges_.clear();

    positions_.clear();
    normals_.clear();
    colors_.clear();
    texcoords_.clear();
    triangles_.clear();
}

void Mesh::Draw(const VBO& vbos, bool wire_frame, int show_texture)
{
    ComputeNormals();

    glPolygonMode(GL_FRONT_AND_BACK, (wire_frame ? GL_LINE : GL_FILL));

    uint size = vertex_number_;
    uint element_num = triangles_.size();

    // position
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
    for (uint i = 0; i < size; ++i)
    {
        positions_[i] = glm::vec3(vec_current_position_[3*i+0], vec_current_position_[3*i+1], vec_current_position_[3*i+2]);
    }
    glBufferData(GL_ARRAY_BUFFER, 3 * size * sizeof(float), &positions_[0], GL_DYNAMIC_DRAW);

    // color
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * size * sizeof(float), &colors_[0], GL_STATIC_DRAW);
    // normal
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * size * sizeof(float), &normals_[0], GL_DYNAMIC_DRAW);
    // texture
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_tbo);
    glBufferData(GL_ARRAY_BUFFER, 2 * size * sizeof(float), &texcoords_[0], GL_STATIC_DRAW);

    // indices
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, element_num * sizeof(unsigned int), &triangles_[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);
    glEnableVertexAttribArray(3);

    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_tbo);
    glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 0, 0);

    glm::mat4 identity = glm::mat4(); // identity matrix
    glUniformMatrix4fv(vbos.m_uniform_transformation, 1, false, &identity[0][0]);

    glUniform1i(vbos.m_uniform_enable_texture, show_texture); // enable/disable texture

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
    glDrawElements(GL_TRIANGLES, element_num, GL_UNSIGNED_INT, 0);

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    glDisableVertexAttribArray(3);
    glUniform1i(vbos.m_uniform_enable_texture, 0); // disable texture

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);    
}

bool Mesh::SaveAsObj(const std::string &filename) const
{
	std::ofstream out(filename.c_str());
	if (!out) { std::cerr << "Can't Save Mesh" << std::endl; }

	std::cout << "Saving as Obj..." << std::endl;

	// vertices
	out << "# " << vertex_number_ << " vertices" << std::endl;
	for (uint i = 0; i<vertex_number_; ++i)
	{
		out << "v" << " ";
		out << vec_current_position_(3 * i + 0) << " ";
		out << vec_current_position_(3 * i + 1) << " ";
		out << vec_current_position_(3 * i + 2) << std::endl;
	}
	out << " " << std::endl;

	// normals
	out << "# " << normals_.size() << " normals" << std::endl;
	for (uint i = 0; i<triangles_.size() / 3; ++i)
	{
		out << "vn" << " ";
		out << normals_[triangles_[3 * i + 0]].x << " ";
		out << normals_[triangles_[3 * i + 0]].y << " ";
		out << normals_[triangles_[3 * i + 0]].z << std::endl;
	}
	out << " " << std::endl;

	// faces
	out << "# " << triangles_.size() / 3 << " faces" << std::endl;
	for (uint i = 0; i<triangles_.size() / 3; ++i)
	{
		out << "f" << " ";
		out << triangles_[3 * i + 0] + 1 << "//" << i + 1 << " ";
		out << triangles_[3 * i + 1] + 1 << "//" << i + 1 << " ";
		out << triangles_[3 * i + 2] + 1 << "//" << i + 1 << std::endl;
	}
	out << " " << std::endl;

	return true;
}

void Mesh::ComputeNormals()
{
    // reset all the normal.
    glm::vec3 zero(0.0);
    for(std::vector<glm::vec3>::iterator n = normals_.begin(); n != normals_.end(); ++n)
    {
        *n = zero;
    }
    // calculate normal for each individual triangle
    uint triangle_num = triangles_.size() / 3;
    uint id0, id1, id2;
    EigenVector3 p0, p1, p2;
    EigenVector3 normal;
    for(uint i = 0; i < triangle_num; ++i)
    {
        id0 = triangles_[3 * i];
        id1 = triangles_[3 * i + 1];
        id2 = triangles_[3 * i + 2];

        p0 = vec_current_position_.block_vector(id0);
        p1 = vec_current_position_.block_vector(id1);
        p2 = vec_current_position_.block_vector(id2);

        normal = (p1-p0).cross(p2-p1);
        normal.normalize();
        glm::vec3 glm_normal = glm::vec3(normal[0], normal[1], normal[2]);

        normals_[id0] += glm_normal;
        normals_[id1] += glm_normal;
        normals_[id2] += glm_normal;
    }
    // re-normalize all the normals.
    for(std::vector<glm::vec3>::iterator n = normals_.begin(); n != normals_.end(); ++n)
    {
        if (glm::length(*n) > EPSILON) // skip if norm is a zero vector
            *n = glm::normalize(*n);
    }
}

#pragma endregion

#pragma region TriangleMesh Class
bool TriangleMesh::Init()
{
	mesh_type_ = kMeshTypeTriangle;

	loaded_mesh_ = new IGLMeshLoader(model_file_path_, kMeshDimension3D, kFileTypeObj, mesh_scaling_);
	if (loaded_mesh_->is_loaded() == false)
	{
		std::cout << "Load mesh error. Using regular Mesh." << std::endl;
		delete loaded_mesh_;

		return false;
	}

    GeneratePositions();
    GenerateTriangles();
    GenerateEdges();
	if (g_enable_bending_constrints)
	{
		GenerateHinges();
	}

    return true;
}

#include <igl/read_triangle_mesh.h>
void TriangleMesh::GeneratePositions()
{
	std::cout << "TetMesh::ParticleList Generating..." << std::endl;

	vertex_number_  = loaded_mesh_->vertex_number();
	system_dimension_ = 3 * vertex_number_;
	std::cout << "m_vertices_number:" << vertex_number_ << std::endl;

	// resize variables for render
    positions_.resize(vertex_number_);
    normals_.resize(vertex_number_);
    colors_.resize(vertex_number_);
    texcoords_.resize(vertex_number_);

    // Assign initial position, velocity and mass to all the vertices.
    // Assign color to all the vertices.
    vec_current_position_.resize(system_dimension_);
    vec_current_velocity_.resize(system_dimension_);
    smat_mass_matrix_.resize(system_dimension_, system_dimension_);
    smat_inv_mass_matrix_.resize(system_dimension_, system_dimension_);
    smat_identity_matrix_.resize(system_dimension_, system_dimension_);

    // Assign initial position / velocity to zero
    vec_current_position_.setZero();
	vec_current_velocity_.setZero();

    for(uint i = 0; i < vertex_number_; ++i)
    {
		vec_current_position_.block_vector(i) = loaded_mesh_->vertex(i);
    }
    
    // Assign mass matrix and an equally sized identity matrix
	ScalarType unit_mass = total_mass_ / system_dimension_;
	ScalarType inv_unit_mass = 1.0 / unit_mass;

    std::vector<SparseMatrixTriplet> i_triplets;		i_triplets.clear();
    std::vector<SparseMatrixTriplet> m_triplets;		m_triplets.clear();
	std::vector<SparseMatrixTriplet> m_inv_triplets;	m_inv_triplets.clear();
    
    for (uint i = 0; i < system_dimension_; ++i)
    {
        i_triplets.push_back(SparseMatrixTriplet(i, i, 1));
        m_triplets.push_back(SparseMatrixTriplet(i, i, unit_mass));
        m_inv_triplets.push_back(SparseMatrixTriplet(i, i, inv_unit_mass));
    }
    smat_mass_matrix_.setFromTriplets(m_triplets.begin(), m_triplets.end());
    smat_inv_mass_matrix_.setFromTriplets(m_inv_triplets.begin(), m_inv_triplets.end());
    smat_identity_matrix_.setFromTriplets(i_triplets.begin(), i_triplets.end());

    // color
    glm::vec3 mesh_color(0.3, 0.8, 1);
    for(uint i = 0; i < vertex_number_; ++i)
    {
        colors_[i] = mesh_color;
    }
}

void TriangleMesh::GenerateTriangles()
{
	uint tri_num = loaded_mesh_->triangle_number();

    triangles_.resize(tri_num * 3);

    for (uint i = 0; i != tri_num; ++i)
    {
		triangles_[3 * i + 0] = loaded_mesh_->triangle(i, 0);
		triangles_[3 * i + 1] = loaded_mesh_->triangle(i, 1);
		triangles_[3 * i + 2] = loaded_mesh_->triangle(i, 2);
    }
}

void TriangleMesh::GenerateEdges()
{
	std::cout << "TetMesh::EdgeList Generating..." << std::endl;

	// consider edge
	SparseMatrix EdgeMatrix(vertex_number_, vertex_number_);
	EdgeMatrix.setZero();

	uint i0, i1;
	for (uint i = 0; i < triangles_.size()/3; ++i)
	{
		uint i_vtx0 = triangles_[3 * i + 0];
		uint i_vtx1 = triangles_[3 * i + 1];
		uint i_vtx2 = triangles_[3 * i + 2];

		i0 = i_vtx0;
		i1 = i_vtx1;
		if (EdgeMatrix.coeff(i0, i1) < EPSILON)
		{
			EdgeMatrix.coeffRef(i0, i1) = 1;
			EdgeMatrix.coeffRef(i1, i0) = 1;
			Edge NewEdge;
			NewEdge.vertex0 = i0;
			NewEdge.vertex1 = i1;
			edges_.push_back(NewEdge);
		}

		i0 = i_vtx1;
		i1 = i_vtx2;
		if (EdgeMatrix.coeff(i0, i1) < EPSILON)
		{
			EdgeMatrix.coeffRef(i0, i1) = 1;
			EdgeMatrix.coeffRef(i1, i0) = 1;
			Edge new_edge;
			new_edge.vertex0 = i0;
			new_edge.vertex1 = i1;
			edges_.push_back(new_edge);
		}

		i0 = i_vtx2;
		i1 = i_vtx0;
		if (EdgeMatrix.coeff(i0, i1) < EPSILON)
		{
			EdgeMatrix.coeffRef(i0, i1) = 1;
			EdgeMatrix.coeffRef(i1, i0) = 1;
			Edge new_edge;
			new_edge.vertex0 = i0;
			new_edge.vertex1 = i1;
			edges_.push_back(new_edge);
		}
	}
}

#include <set>
void TriangleMesh::GenerateHinges()
{
	std::cout << "TetMesh::HingeList Generating..." << std::endl;

	bool showInfo = false;

	SparseMatrix HingeMatrix(triangles_.size() / 3, triangles_.size() / 3);
	HingeMatrix.setZero();

	for (uint i = 0; i < triangles_.size() / 3; ++i)
	{
		// index list of tri i
		std::vector<uint> itri_i;	itri_i.resize(3);
		itri_i[0] = triangles_[3 * i + 0];
		itri_i[1] = triangles_[3 * i + 1];
		itri_i[2] = triangles_[3 * i + 2];

#ifdef OMEP_MP
#pragma omp parallel for private(j)
#endif // OMEP_MP
		for (uint j = 0; j < triangles_.size() / 3; ++j)
		{
			//printf("Roop:(%d, %d)\n", i, j);

			// skip invaluable conbination
			if (i == j)
			{
				if (showInfo) { printf("Skip:(%d, %d)\n", i, j); }
				continue;
			}
			if (HingeMatrix.coeff(i, j) > EPSILON)
			{
				if (showInfo) { printf("Skip:(%d, %d)\n", i, j); }
				continue;
			}

			// index list of tri j
			std::vector<uint> itri_j;	itri_j.resize(3);
			itri_j[0] = triangles_[3 * j + 0];
			itri_j[1] = triangles_[3 * j + 1];
			itri_j[2] = triangles_[3 * j + 2];

			std::vector<uint> aTriIndex(6, 0);
			aTriIndex[0] = itri_i[0];
			aTriIndex[1] = itri_i[1];
			aTriIndex[2] = itri_i[2];
			aTriIndex[3] = itri_j[0];
			aTriIndex[4] = itri_j[1];
			aTriIndex[5] = itri_j[2];

			// skip invaluable conbination
			std::set<uint> set(aTriIndex.begin(), aTriIndex.end());
			assert(aTriIndex.size() <= 6);
			//if (aTriIndex.size() < 4) continue;
			if (aTriIndex.size() > 4)
			{
				if (showInfo) { printf("Skip:(%d, %d)\n", i, j); }
				continue;
			}

			// now this is valuable combination
			std::vector<uint> aSharedVtx;		aSharedVtx.clear();
			std::vector<uint> aNotSharedVtx;	aNotSharedVtx.clear();

			for (auto itr = set.begin(); itr != set.end(); ++itr)
			{
				int number = std::count(aTriIndex.begin(), aTriIndex.end(), *itr);
				if (number == 2) { aSharedVtx.push_back(*itr); }
				else if (number == 1) { aNotSharedVtx.push_back(*itr); }
			}
			assert(aSharedVtx.size() == 2);
			assert(aNotSharedVtx.size() == 2);

			printf("NewHinge:(%d, %d)\n", i, j);

			// new hinge
			HingeMatrix.coeffRef(i, j) = 1;
			HingeMatrix.coeffRef(j, i) = 1;

			Hinge NewHinge;
			NewHinge.triangle0 = i;
			NewHinge.triangle1 = j;

			NewHinge.vertex0 = aNotSharedVtx[0];
			NewHinge.vertex1 = aNotSharedVtx[1];
			NewHinge.vertex2 = aSharedVtx[0];
			NewHinge.vertex3 = aSharedVtx[1];

			hinges_.push_back(NewHinge);
		}// !for j
	}// !for i
}

#pragma endregion