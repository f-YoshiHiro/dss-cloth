#ifndef _PRIMITIVE_H_
#define _PRIMITIVE_H_

#include "global_headers.h"
#include "math_headers.h"
//#include "bvh.h"
#include "opengl_headers.h"
//#include "bvh.h"

//class AABB;
//class NodeBVH;

// TODO: add more primitives here.
enum PrimitiveType {PLANE, SPHERE, CUBE, OBJMESH};
class Primitive
{
public:
    Primitive(const PrimitiveType& t) : m_type(t), m_pos(glm::vec3(0,0,0)), m_vel(glm::vec3(0,0,0)),  m_has_vel(false), m_has_gravity(false), m_previous_pos(glm::vec3(0,0,0)) {init_visualization();};
    Primitive(const PrimitiveType& t, glm::vec3 pos) : m_type(t), m_pos(pos), m_vel(glm::vec3(0,0,0)), m_has_vel(false), m_has_gravity(false), m_previous_pos(pos) {init_visualization();};
    Primitive(const PrimitiveType& t, glm::vec3 pos, glm::vec3 vel) : m_type(t), m_pos(pos), m_vel(vel), m_has_vel(true), m_has_gravity(true), m_previous_pos(pos) {init_visualization();};
    Primitive(const PrimitiveType& t, glm::vec3 pos, glm::vec3 vel, bool has_gravity) : m_type(t), m_pos(pos), m_vel(vel), m_has_vel(true), m_has_gravity(has_gravity), m_previous_pos(pos) {init_visualization();};
    Primitive(const Primitive& other) : 
        m_type(other.m_type), 
        m_positions(other.m_positions), m_colors(other.m_colors), m_normals(other.m_normals),
        m_indices(other.m_indices),
        m_pos(other.m_pos), m_vel(other.m_vel), m_previous_pos(other.m_previous_pos)
    {
    }
    virtual ~Primitive()
    {
        m_positions.clear();
        m_colors.clear();
        m_normals.clear();
        m_indices.clear();
    }

    PrimitiveType type() const
    {
        return m_type;
    }

    virtual void move_to(const glm::vec3& target) {m_pos = target;}
    virtual bool StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist) {return false;}

    virtual void change_color(const glm::vec3& color);
    virtual void Draw(const VBO& vbos);

protected:
    virtual void init_visualization() {}
	virtual void InitCollision(){}
public:
    glm::vec3 m_pos;
    glm::vec3 m_previous_pos;
    glm::vec3 m_vel;
    bool m_has_vel;
    bool m_has_gravity;

protected:
    PrimitiveType m_type;
    std::vector<glm::vec3> m_positions, m_colors, m_normals;
    std::vector<unsigned short> m_indices;

	// for collision
	EigenVectorXs vec_vertices_;
	std::vector<uint> triangles_;

	// for collision handling
	int index_root_;	// index of root node
	//std::vector<NodeBVH> node_bvhs;
	//std::vector<AABB>    aabb_bvhs;
};

class Plane : public Primitive
{
public:
    Plane() : Primitive(PLANE), m_normal(glm::vec3(0.0, 1.0, 0.0)) {init_visualization();};
    Plane(const glm::vec3 norm, float value) : Primitive(PLANE, glm::vec3(0.0, value, 0.0)), m_normal(norm) {init_visualization();};
    Plane(const Plane& other) : Primitive(other), m_normal(other.m_normal){};
    virtual ~Plane() {};

    virtual bool StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist);
        
protected:
    virtual void init_visualization();
protected:
    glm::vec3 m_normal;
};

class Sphere : public Primitive
{
public:
    Sphere() : Primitive(SPHERE), m_radius(DEFAULT_SELECTION_RADIUS) {init_visualization();};
    Sphere(float radius) : Primitive(SPHERE), m_radius(radius) {init_visualization();};
    Sphere(const glm::vec3 pos, float radius) : Primitive(SPHERE, pos), m_radius(radius) {init_visualization();};
    Sphere(const glm::vec3 pos, const glm::vec3 vel, float radius) : Primitive(SPHERE, pos, vel), m_radius(radius) {init_visualization();};
    Sphere(const glm::vec3 pos, const glm::vec3 vel, bool has_gravity, float radius) : Primitive(SPHERE, pos, vel, has_gravity), m_radius(radius) {init_visualization();};
    Sphere(const Sphere& other) : Primitive(other), m_radius(other.m_radius) {};
    virtual ~Sphere() {};

    virtual bool StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist);
protected:
    virtual void init_visualization();
protected:
    float m_radius;
};

class Cube : public Primitive
{
public:
    Cube() : Primitive(CUBE), m_hf_dims(glm::vec3(1.0f, 1.0f, 1.0f)) {init_visualization();};
    Cube(float hf_x, float hf_y, float hf_z) : Primitive(CUBE), m_hf_dims(glm::vec3(hf_x, hf_y, hf_z)) {init_visualization();};
    Cube(glm::vec3 hf_dims) : Primitive(CUBE), m_hf_dims(hf_dims) {init_visualization();};
    Cube(const glm::vec3 pos,glm::vec3 hf_dims) : Primitive(CUBE, pos), m_hf_dims(hf_dims) {init_visualization();};
    Cube(const Cube& other) : Primitive(other), m_hf_dims(other.m_hf_dims) {};
    virtual ~Cube() {};

    virtual void move_to(const glm::vec3& target);
    virtual bool StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist);
protected:
    virtual void init_visualization();
protected:
    glm::vec3 m_hf_dims;
};

class ObjMesh : public Primitive
{
public:
    ObjMesh(char* filename) : Primitive(OBJMESH), m_scaling(1.0) 
	{
		read_from_file(filename);
		InitCollision();
	};
    ObjMesh(char* filename, float scaling) : Primitive(OBJMESH), m_scaling(scaling) 
	{
		read_from_file(filename);
		InitCollision();
	};
    ObjMesh(char* filename, const glm::vec3 pos, float scaling) : Primitive(OBJMESH, pos), m_scaling(scaling) 
	{
		read_from_file(filename);
		InitCollision();
	};
    ObjMesh(const ObjMesh& other) : Primitive(other), m_scaling(other.m_scaling) {};
    virtual ~ObjMesh() {};

    virtual bool StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist);
protected:
    void read_from_file(char* filename);
	virtual void InitCollision();
protected:
    char* m_filename;
    float m_scaling;
};

#endif