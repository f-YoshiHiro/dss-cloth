#pragma warning( disable : 4996)

#include "simulation.h"
#include "timer_wrapper.h"

//----------State Control--------------------//
extern bool g_enable_bending_constrints;

Simulation::Simulation()

{
}

Simulation::~Simulation()
{
    clearConstraints();
}

void Simulation::Reset()
{    
    m_inertia_y.resize(m_mesh->system_dimension_);
    m_external_force.resize(m_mesh->system_dimension_);

    setupConstraints();
    SetReprefactorFlag();

    m_selected_attachment_constraint = NULL;
}

void Simulation::Update()
{
    // update inertia term
    calculateInertiaY();

    // update external force
    calculateExternalForce();

    // update cloth
    switch (m_integration_method)
    {
    case INTEGRATION_LOCAL_GLOBAL:
        integrateOptimizationMethod();
        break;
    }

    // Add collision detection here using pos_next;
    EigenVectorXs penetration = collisionDetection(m_mesh->vec_current_position_);
    m_mesh->vec_current_position_ -= penetration;

    // update velocity and damp
    dampVelocity();
}

void Simulation::DrawConstraints(const VBO& vbos)
{
    for (std::vector<Constraint*>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
    {
        (*it)->Draw(vbos);
    }
}

ScalarType Simulation::TryToSelectAttachmentConstraint(const EigenVector3& p0, const EigenVector3& dir)
{
    ScalarType ray_point_dist;
    ScalarType min_dist = 100.0;
    AttachmentConstraint* best_candidate = NULL;

    bool current_state_on = false;
    for (std::vector<Constraint*>::iterator c = m_constraints.begin(); c != m_constraints.end(); ++c)
    {
        AttachmentConstraint* ac;
        if (ac = dynamic_cast<AttachmentConstraint*>(*c)) // is attachment constraint
        {
            ray_point_dist = ((ac->GetFixedPoint()-p0).cross(dir)).norm();
            if (ray_point_dist < min_dist)
            {
                min_dist = ray_point_dist;
                best_candidate = ac;
            }
        }
    }
    // exit if no one fits
    if (min_dist > DEFAULT_SELECTION_RADIUS)
    {
        UnselectAttachmentConstraint();

        return -1;
    }
    else
    {
        SelectAtttachmentConstraint(best_candidate);
        EigenVector3 fixed_point_temp = m_mesh->vec_current_position_.block_vector(m_selected_attachment_constraint->GetConstrainedVertexIndex());

        return (fixed_point_temp-p0).dot(dir); // this is m_cached_projection_plane_distance
    }
}

bool Simulation::TryToToggleAttachmentConstraint(const EigenVector3& p0, const EigenVector3& dir)
{
    EigenVector3 p1;

    ScalarType ray_point_dist;
    ScalarType min_dist = 100.0;
    unsigned int best_candidate = 0;
    // first pass: choose nearest point
    for (unsigned int i = 0; i != m_mesh->vertex_number_; i++)
    {
        p1 = m_mesh->vec_current_position_.block_vector(i);
        
        ray_point_dist = ((p1-p0).cross(dir)).norm();
        if (ray_point_dist < min_dist)
        {
            min_dist = ray_point_dist;
            best_candidate = i;
        }
    }
    for (std::vector<Constraint*>::iterator c = m_constraints.begin(); c != m_constraints.end(); ++c)
    {
        AttachmentConstraint* ac;
        if (ac = dynamic_cast<AttachmentConstraint*>(*c)) // is attachment constraint
        {
            ray_point_dist = ((ac->GetFixedPoint()-p0).cross(dir)).norm();
            if (ray_point_dist < min_dist)
            {
                min_dist = ray_point_dist;
                best_candidate = ac->GetConstrainedVertexIndex();
            }
        }
    }
    // exit if no one fits
    if (min_dist > DEFAULT_SELECTION_RADIUS)
    {
        return false;
    }
    // second pass: toggle that point's fixed position constraint
    bool current_state_on = false;
    for (std::vector<Constraint*>::iterator c = m_constraints.begin(); c != m_constraints.end(); ++c)
    {
        AttachmentConstraint* ac;
        if (ac = dynamic_cast<AttachmentConstraint*>(*c)) // is attachment constraint
        {
            if (ac->GetConstrainedVertexIndex() == best_candidate)
            {
                current_state_on = true;
                m_constraints.erase(c);
                break;
            }
        }
    }
    if (!current_state_on)
    {
        AddAttachmentConstraint(best_candidate);
    }

    return true;
}

void Simulation::SelectAtttachmentConstraint(AttachmentConstraint* ac)
{
    m_selected_attachment_constraint = ac;
    m_selected_attachment_constraint->Select();
}

void Simulation::UnselectAttachmentConstraint()
{
    if (m_selected_attachment_constraint)
    {
        m_selected_attachment_constraint->UnSelect();
    }
    m_selected_attachment_constraint = NULL;
}

void Simulation::AddAttachmentConstraint(unsigned int vertex_index)
{
    AttachmentConstraint* ac = new AttachmentConstraint(&m_stiffness_attachment, vertex_index, m_mesh->vec_current_position_.block_vector(vertex_index));
    m_constraints.push_back(ac);
}

void Simulation::MoveSelectedAttachmentConstraintTo(const EigenVector3& target)
{
    if (m_selected_attachment_constraint)
        m_selected_attachment_constraint->SetFixedPoint(target);
}

void Simulation::clearConstraints()
{
    for (unsigned int i = 0; i < m_constraints.size(); ++i)
    {
        delete m_constraints[i];
    }
    m_constraints.clear();
}

void Simulation::setupConstraints()
{
    clearConstraints();

    switch(m_mesh->mesh_type_)
    {
		case kMeshTypeTriangle:
		{
			// generate stretch constraints. assign a stretch constraint for each edge.
			for (std::vector<Edge>::iterator e = m_mesh->edges_.begin(); e != m_mesh->edges_.end(); ++e)
			{
				EigenVector3 p1, p2;
				p1 = m_mesh->vec_current_position_.block_vector(e->vertex0);
				p2 = m_mesh->vec_current_position_.block_vector(e->vertex1);
				SpringConstraint *c = new SpringConstraint(&m_stiffness_stretch, e->vertex0, e->vertex1, (p1 - p2).norm());
				m_constraints.push_back(c);
			}

			if (g_enable_bending_constrints)
			{
				// generate bending constraints.
				for (std::vector<Hinge>::iterator h = m_mesh->hinges_.begin(); h != m_mesh->hinges_.end(); ++h)
				{
					EigenVector3 p1, p2;
					p1 = m_mesh->vec_current_position_.block_vector(h->vertex0);
					p2 = m_mesh->vec_current_position_.block_vector(h->vertex1);
					SpringConstraint *c = new SpringConstraint(&m_stiffness_bending, h->vertex0, h->vertex1, (p1 - p2).norm());
					m_constraints.push_back(c);
				}
			}

        }
        break;
    }
}

void Simulation::dampVelocity()
{
    if (std::abs(m_damping_coefficient) < EPSILON)
        return;

    // post-processing damping
    EigenVector3 pos_mc(0.0, 0.0, 0.0), vel_mc(0.0, 0.0, 0.0);
    unsigned int i, size;
    ScalarType denominator(0.0), mass(0.0);
    size = m_mesh->vertex_number_;
    for(i = 0; i < size; ++i)
    {
        mass = m_mesh->smat_mass_matrix_.coeff(i*3, i*3);

        pos_mc += mass * m_mesh->vec_current_position_.block_vector(i);
        vel_mc += mass * m_mesh->vec_current_velocity_.block_vector(i);
        denominator += mass;
    }
    assert(denominator != 0.0);
    pos_mc /= denominator;
    vel_mc /= denominator;

    EigenVector3 angular_momentum(0.0, 0.0, 0.0), r(0.0, 0.0, 0.0);
    EigenMatrix3 inertia, r_mat;
    inertia.setZero(); r_mat.setZero();

    for(i = 0; i < size; ++i)
    {
        mass = m_mesh->smat_mass_matrix_.coeff(i*3, i*3);

        r = m_mesh->vec_current_position_.block_vector(i) - pos_mc;
        angular_momentum += r.cross(mass * m_mesh->vec_current_velocity_.block_vector(i));

        //r_mat = EigenMatrix3(0.0,  r.z, -r.y,
        //                    -r.z, 0.0,  r.x,
        //                    r.y, -r.x, 0.0);

        r_mat.coeffRef(0, 1) = r[2];
        r_mat.coeffRef(0, 2) = -r[1];
        r_mat.coeffRef(1, 0) = -r[2];
        r_mat.coeffRef(1, 2) = r[0];
        r_mat.coeffRef(2, 0) = r[1];
        r_mat.coeffRef(2, 1) = -r[0];

        inertia += r_mat * r_mat.transpose() * mass;
    }
    EigenVector3 angular_vel = inertia.inverse() * angular_momentum;

    EigenVector3 delta_v(0.0, 0.0, 0.0);
    for(i = 0; i < size; ++i)
    {
        r = m_mesh->vec_current_position_.block_vector(i) - pos_mc;
        delta_v = vel_mc + angular_vel.cross(r) - m_mesh->vec_current_velocity_.block_vector(i);     
        m_mesh->vec_current_velocity_.block_vector(i) += m_damping_coefficient * delta_v;
    }
}

void Simulation::calculateInertiaY()
{
    m_inertia_y = m_mesh->vec_current_position_ + m_mesh->vec_current_velocity_ * m_h;
}

void Simulation::calculateExternalForce()
{
    m_external_force.resize(m_mesh->system_dimension_);
    m_external_force.setZero();

    // gravity
    for (unsigned int i = 0; i < m_mesh->vertex_number_; ++i)
    {
        m_external_force[3*i+1] += -m_gravity_constant;
    }

    m_external_force = m_mesh->smat_mass_matrix_ * m_external_force;
}

EigenVectorXs Simulation::collisionDetection(const EigenVectorXs x)
{
    // Naive implementation of collision detection
    EigenVectorXs penetration(m_mesh->system_dimension_);
    penetration.setZero();
    EigenVector3 normal;
    ScalarType dist;

    for (unsigned int i = 0; i < m_mesh->vertex_number_; ++i)
    {
        EigenVector3 xi = x.block_vector(i);

        if (m_scene->StaticIntersectionTest(xi, normal, dist))
        {
            penetration.block_vector(i) += (dist) * normal;
        }
    }

    return penetration;
}

void Simulation::integrateOptimizationMethod()
{
    // take a initial guess
    EigenVectorXs pos_next = m_inertia_y;

    // while loop until converge or exceeds maximum iterations
    bool converge = false;

    for (unsigned int iteration_num = 0; !converge && iteration_num < m_iterations_per_frame; ++iteration_num)
    {
        switch (m_integration_method)
        {
        case INTEGRATION_LOCAL_GLOBAL:
            converge = integrateLocalGlobalOneIteration(pos_next);
            break;
        }
    }

    // update q_{n+1}
    updatePosAndVel(pos_next);
}

bool Simulation::integrateLocalGlobalOneIteration(EigenVectorXs& x)
{
    prefactorize(PREFACTOR_M_PLUS_H2L);

    EigenVectorXs d;
    evaluateDVector(x, d);

    EigenVectorXs b = m_mesh->smat_mass_matrix_ * m_inertia_y + m_h*m_h*(m_J_matrix*d+m_external_force);

    x = m_prefactored_solver[PREFACTOR_M_PLUS_H2L].solve(b);

    return false;
}

// for local global method only
void Simulation::evaluateDVector(const EigenVectorXs& x, EigenVectorXs& d)
{
    d.resize(m_constraints.size()*3);
    d.setZero();

    for (unsigned int index = 0; index < m_constraints.size(); ++index)
    {
        m_constraints[index]->EvaluateDVector(index, x, d);
    }
}
void Simulation::evaluateJMatrix(SparseMatrix& J)
{
    J.resize(m_mesh->system_dimension_, m_constraints.size()*3);
    std::vector<SparseMatrixTriplet> J_triplets;
    J_triplets.clear();

    for (unsigned int index = 0; index < m_constraints.size(); ++index)
    {
        m_constraints[index]->EvaluateJMatrix(index, J_triplets);
    }
    J.setFromTriplets(J_triplets.begin(), J_triplets.end());
}


#pragma region matrices and prefactorization
void Simulation::setWeightedLaplacianMatrix()
{
    m_weighted_laplacian.resize(m_mesh->system_dimension_, m_mesh->system_dimension_);
    std::vector<SparseMatrixTriplet> l_triplets;
    l_triplets.clear();

    for (std::vector<Constraint*>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
    {
        (*it)->EvaluateWeightedLaplacian(l_triplets);
    }

    m_weighted_laplacian.setFromTriplets(l_triplets.begin(), l_triplets.end());
}

void Simulation::setJMatrix()
{
    m_J_matrix;

    evaluateJMatrix(m_J_matrix);
}

void Simulation::prefactorize(PrefactorType type)
{
    if (m_prefatorization_flag[type] == false)
    {
        SparseMatrix A;
        ScalarType h2 = m_h*m_h;

        // choose matrix
        switch (type)
        {
        case PREFACTOR_L:
            setWeightedLaplacianMatrix();
            setJMatrix();
            A = m_weighted_laplacian;
            break;
        case PREFACTOR_M_PLUS_H2L:
            setWeightedLaplacianMatrix();
            setJMatrix();
            A = m_mesh->smat_mass_matrix_ + h2 * m_weighted_laplacian;
            break;
        }

        factorizeDirectSolverLLT(A, m_prefactored_solver[type]);
        m_prefatorization_flag[type] = true;
    }
}

#pragma endregion

#pragma region utilities
void Simulation::updatePosAndVel(const EigenVectorXs& new_pos)
{
    m_mesh->vec_current_velocity_ = (new_pos - m_mesh->vec_current_position_)/m_h;
    m_mesh->vec_current_position_ = new_pos;
}

void Simulation::factorizeDirectSolverLLT(const SparseMatrix& A, Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper>& lltSolver, char* warning_msg)
{
    SparseMatrix A_prime = A;
    lltSolver.analyzePattern(A_prime);
    lltSolver.factorize(A_prime);
    ScalarType Regularization = 0.00001;
    bool success = true;
    while (lltSolver.info() != Eigen::Success)
    {
        Regularization *= 10;
        A_prime = A_prime + Regularization*m_mesh->smat_identity_matrix_;
        lltSolver.factorize(A_prime);
        success = false;
    }
    if (!success)
        std::cout << "Warning: " << warning_msg <<  " adding "<< Regularization <<" identites.(llt solver)" << std::endl;
}

void Simulation::factorizeDirectSolverLDLT(const SparseMatrix& A, Eigen::SimplicialLDLT<SparseMatrix, Eigen::Upper>& ldltSolver, char* warning_msg)
{
    SparseMatrix A_prime = A;
    ldltSolver.analyzePattern(A_prime);
    ldltSolver.factorize(A_prime);
    ScalarType Regularization = 0.00001;
    bool success = true;
    while (ldltSolver.info() != Eigen::Success)
    {
        Regularization *= 10;
        A_prime = A_prime + Regularization*m_mesh->smat_identity_matrix_;
        ldltSolver.factorize(A_prime);
        success = false;
    }
    if (!success)
        std::cout << "Warning: " << warning_msg <<  " adding "<< Regularization <<" identites.(ldlt solver)" << std::endl;
}

void Simulation::generateRandomVector(const unsigned int size, EigenVectorXs& x)
{
    x.resize(size);

    for (unsigned int i = 0; i < size; i++)
    {
        x(i) = ((ScalarType)(rand())/(ScalarType)(RAND_MAX+1)-0.5)*2;
    }
}

#pragma endregion
