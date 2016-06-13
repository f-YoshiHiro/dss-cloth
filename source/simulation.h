#ifndef _SIMULATION_H_
#define _SIMULATION_H_

#include <vector>

#include "global_headers.h"
#include "anttweakbar_wrapper.h"
#include "mesh.h"
#include "constraint.h"
#include "scene.h"

class Mesh;
class AntTweakBarWrapper;

typedef enum
{
    PREFACTOR_L,
    PREFACTOR_M_PLUS_H2L,
    PREFACTOR_TOTAL_NUM

} PrefactorType;

typedef enum
{
    INTEGRATION_LOCAL_GLOBAL,
    INTEGRATION_TOTAL_NUM

} IntegrationMethod;

class Simulation
{
    friend class AntTweakBarWrapper;

public:
    Simulation();
    virtual ~Simulation();

    void Reset();
    void Update();
    void DrawConstraints(const VBO& vbos);

    // select/unselect/move attachment constratins
    ScalarType TryToSelectAttachmentConstraint(const EigenVector3& p0, const EigenVector3& dir); // return ray_projection_plane_distance if hit; return -1 otherwise.
    bool TryToToggleAttachmentConstraint(const EigenVector3& p0, const EigenVector3& dir); // true if hit some vertex/constraint
    void SelectAtttachmentConstraint(AttachmentConstraint* ac);
    void UnselectAttachmentConstraint();
    void AddAttachmentConstraint(unsigned int vertex_index); // add one attachment constraint at vertex_index
    void MoveSelectedAttachmentConstraintTo(const EigenVector3& target); // move selected attachement constraint to target

    // inline functions
    inline void SetReprefactorFlag() 
    {
        for (unsigned int i = 0; i < PREFACTOR_TOTAL_NUM; ++i)
        {
            m_prefatorization_flag[i] = false;
        }
    }
    inline void SetMesh(Mesh* mesh) {m_mesh = mesh;}
    inline void SetScene(Scene* scene) {m_scene = scene;}
    
protected:

    // simulation constants
    ScalarType m_h; // time_step

    // simulation constants
    ScalarType m_gravity_constant;
    ScalarType m_stiffness_attachment;
    ScalarType m_stiffness_stretch;
    ScalarType m_stiffness_bending;
    ScalarType m_damping_coefficient;

    // integration method
    IntegrationMethod m_integration_method;

    // key simulation components: mesh and scene
    Mesh *m_mesh;
    Scene *m_scene;
    // key simulation components: constraints
    std::vector<Constraint*> m_constraints;
    AttachmentConstraint* m_selected_attachment_constraint;

    // inertia term
    EigenVectorXs m_inertia_y;

    // external force (gravity, wind, etc...)
    EigenVectorXs m_external_force;

    // for optimization method, number of iterations
    unsigned int m_iterations_per_frame;

    // for prefactorization
    SparseMatrix m_weighted_laplacian;
    SparseMatrix m_J_matrix;
    bool m_prefatorization_flag[PREFACTOR_TOTAL_NUM];
    Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper> m_prefactored_solver[PREFACTOR_TOTAL_NUM];

private:

    // main update sub-routines
    void clearConstraints(); // cleanup all constraints
    void setupConstraints(); // initialize constraints
    void dampVelocity(); // damp velocity at the end of each iteration.
    void calculateInertiaY(); // calculate the inertia term: y = current_pos + current_vel*h
    void calculateExternalForce(); // wind force is propotional to the area of triangles projected on the tangential plane
    EigenVectorXs collisionDetection(const EigenVectorXs x); // detect collision and return a vector of penetration

    void integrateOptimizationMethod();

    // all those "OneIteration" functions will be called in a loop
    // x is initially passed as the initial guess of the next postion (i.e. inertia term): x = y = current_pos + current_vel*h
    // x will be changed during these subroutines in EVERY iteration
    // the final value of x will be the next_pos that we used to update all vertices.
    bool integrateLocalGlobalOneIteration(EigenVectorXs& x); // our method

    // for local global method only
    void evaluateDVector(const EigenVectorXs& x, EigenVectorXs& d); // d-vector will be evaluate every iteration
    void evaluateJMatrix(SparseMatrix& J); // J matrix is only dependent on connectivity and stiffness, so it is evaluated only once.

    // matrices and prefactorizations
    void setWeightedLaplacianMatrix();
    void setJMatrix();
    void prefactorize(PrefactorType type);

    // utility functions
    void updatePosAndVel(const EigenVectorXs& new_pos); // current_vel = (next_pos-current_pos)/h; current_pos = next_pos; 
    void factorizeDirectSolverLLT(const SparseMatrix& A, Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper>& lltSolver, char* warning_msg = ""); // factorize matrix A using LLT decomposition
    void factorizeDirectSolverLDLT(const SparseMatrix& A, Eigen::SimplicialLDLT<SparseMatrix, Eigen::Upper>& ldltSolver, char* warning_msg = ""); // factorize matrix A using LDLT decomposition
    void generateRandomVector(const unsigned int size, EigenVectorXs& x); // generate random vector varing from [-1 1].
};

#endif