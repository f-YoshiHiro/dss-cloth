#ifndef _MESH_H_
#define _MESH_H_

#include <vector>

#include "opengl_headers.h"
#include "global_headers.h"
#include "math_headers.h"
#include "io_mesh.h"
#include "anttweakbar_wrapper.h"
#include "simulation.h"
#include "camera.h"
#include "primitive.h"

//---------- Forward Declaration ----------//
class Camera;
class AntTweakBarWrapper;
class Simulation;

//---------- Enumeration ----------//
typedef enum MeshType
{
    kMeshTypeTriangle,
    kMeshTypeTotalNumber
};

//---------- Structure ----------//
struct Edge
{
    uint vertex0,   vertex1;     
    uint triangle0, triangle1; 
};

struct Hinge
{
	uint vertex0,   vertex1,   vertex2,   vertex3;
	uint triangle0, triangle1;
};

//---------- Class ----------//
class Mesh
{
		friend class AntTweakBarWrapper;
		friend class Simulation;
		friend class Camera;

	public:
		Mesh() : mesh_type_() { }
		explicit Mesh(MeshType mesh_type) : mesh_type_(mesh_type) { }
		virtual ~Mesh() {Cleanup();}

		void Reset();
	    virtual bool Init() {std::cout << "Warning: reach base class virtual init function." << std::endl; return false;}
	    virtual void Cleanup();

	    virtual void Draw(const VBO& vbos, bool wire_frame = false, int show_texture = 0);

		bool SaveAsObj(const std::string &filename) const;

		// accesser / mutater
		virtual inline uint vertex_number() const { return vertex_number_; }
		virtual inline uint system_dimension() const { return system_dimension_; }
		inline MeshType mesh_type() { return mesh_type_; }

	protected:
		// initialize every pos / vel / mass / color.
		virtual void GeneratePositions() { std::cout << "Warning: reach base class virtual function." << std::endl; }
		virtual void GenerateTriangles() { std::cout << "Warning: reach base class virtual function." << std::endl; }
		virtual void GenerateEdges()  { std::cout << "Warning: reach base class virtual function." << std::endl; }
		virtual void GenerateHinges() { std::cout << "Warning: reach base class virtual function." << std::endl; }

		void ComputeNormals();


	    MeshType mesh_type_;

	    uint vertex_number_;    // m
	    uint system_dimension_; // 3m

	    // vertices positions/previous positions/mass
	    EigenVectorXs vec_current_position_; // 1x3m
	    EigenVectorXs vec_current_velocity_; // 1x3m
	    SparseMatrix  smat_mass_matrix_;     // 3mx3m
	    SparseMatrix  smat_inv_mass_matrix_; // 3mx3m
	    SparseMatrix  smat_identity_matrix_; // 3mx3m

	    // for generating constraints.
	    std::vector<Edge>  edges_;
		std::vector<Hinge> hinges_;

	    // for visualization
	    std::vector<glm::vec3> positions_;
	    std::vector<glm::vec3> normals_;
	    std::vector<glm::vec3> colors_;
	    std::vector<glm::vec2> texcoords_;
	    std::vector<uint>      triangles_;

	    // for all
	    ScalarType total_mass_;

	    // put them here because of anttweakbar
	    char model_file_path_[256];
	    ScalarType mesh_scaling_;

	private:
		DISALLOW_COPY_AND_ASSIGN(Mesh);
};

class TriangleMesh : public Mesh
{
		friend class AntTweakBarWrapper;
		friend class Simulation;

	public:
		TriangleMesh() : 
			Mesh(kMeshTypeTriangle),
			loaded_mesh_(NULL) 
		{ }

		virtual ~TriangleMesh() { if (loaded_mesh_) delete loaded_mesh_;  }

		virtual bool Init();

	protected:
		virtual void GeneratePositions();// initialize every particle pos / vel / mass / color.
		virtual void GenerateTriangles();
		virtual void GenerateEdges();
		virtual void GenerateHinges();


		// loaded file
		IGLMeshLoader* loaded_mesh_;

	private:
		DISALLOW_COPY_AND_ASSIGN(TriangleMesh);
};

#endif