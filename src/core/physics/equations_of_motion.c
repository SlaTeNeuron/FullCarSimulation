#include "core/physics/equations_of_motion.h"
#include "vehicle/vehicle.h"
#include <stdlib.h>
#include <string.h>

//-------------------------
// Internal State
//-------------------------

struct EquationsOfMotion {
    int num_dofs;            // Number of degrees of freedom
    vde_real* mass_matrix;   // Mass matrix (num_dofs x num_dofs)
    vde_real* force_vector;  // Force vector (num_dofs)
    vde_real* accelerations; // Computed accelerations (num_dofs)
};

//-------------------------
// Lifecycle
//-------------------------

/**
 * Create equations of motion structure
 * 
 * Allocates storage for the mass matrix (M), force vector (Q), and
 * acceleration vector (q_ddot) for a system with the specified DOFs.
 * 
 * Guiggiani Reference: Section 3.6, 3.11
 *   Equations: M * q_ddot = Q
 * 
 * Input:
 *   - num_dofs: Number of degrees of freedom (must be > 0)
 * 
 * Output:
 *   - Returns pointer to EquationsOfMotion, or NULL on failure
 */
EquationsOfMotion* eom_create(int num_dofs) {
    if (num_dofs <= 0) return NULL;
    
    EquationsOfMotion* eom = (EquationsOfMotion*)malloc(sizeof(EquationsOfMotion));
    if (!eom) return NULL;
    
    eom->num_dofs = num_dofs;
    eom->mass_matrix = (vde_real*)calloc(num_dofs * num_dofs, sizeof(vde_real));
    eom->force_vector = (vde_real*)calloc(num_dofs, sizeof(vde_real));
    eom->accelerations = (vde_real*)calloc(num_dofs, sizeof(vde_real));
    
    if (!eom->mass_matrix || !eom->force_vector || !eom->accelerations) {
        eom_destroy(eom);
        return NULL;
    }
    
    return eom;
}

/**
 * Destroy equations of motion and free resources
 * 
 * Input:
 *   - eom: Equations of motion to destroy (can be NULL)
 */
void eom_destroy(EquationsOfMotion* eom) {
    if (!eom) return;
    
    if (eom->mass_matrix) free(eom->mass_matrix);
    if (eom->force_vector) free(eom->force_vector);
    if (eom->accelerations) free(eom->accelerations);
    free(eom);
}

//-------------------------
// Building Equations
//-------------------------

/**
 * Build equations of motion from vehicle state
 * 
 * Constructs the mass matrix M and force vector Q from the current
 * vehicle state. This implements Guiggiani's EQUILIBRIUM equations.
 * 
 * Guiggiani Reference: Section 3.6, 3.11, 9.6
 *   Linear momentum: m * (u_dot - v*r + w*q) = X
 *   Angular momentum: I*omega_dot + omega × (I*omega) = M
 * 
 * Input:
 *   - eom: Equations of motion structure (must be non-NULL)
 *   - vehicle: Vehicle state (must be non-NULL)
 * 
 * Output:
 *   - Fills eom->mass_matrix and eom->force_vector
 * 
 * Functionality:
 *   1. Extract mass and inertia from vehicle
 *   2. Build mass matrix (diagonal for unconstrained system)
 *   3. Compute forces (tire forces, aero, gravity, etc.)
 *   4. Add gyroscopic terms (omega × I*omega)
 */
void eom_build_from_vehicle(EquationsOfMotion* eom, const Vehicle* vehicle) {
    if (!eom || !vehicle) return;
    
    // TODO: Implement equation building
    // 1. Extract mass and inertia from vehicle
    // 2. Build mass matrix
    // 3. Compute total forces and moments
    // 4. Account for gyroscopic terms
}

//-------------------------
// Accessors
//-------------------------

/**
 * Get the mass matrix
 * 
 * Input:
 *   - eom: Equations of motion (must be non-NULL)
 *   - out_matrix: Output buffer (must have space for num_dofs² elements)
 * 
 * Output:
 *   - Copies mass matrix to out_matrix in row-major order
 */
void eom_get_mass_matrix(const EquationsOfMotion* eom, vde_real* out_matrix) {
    if (!eom || !out_matrix) return;
    
    int size = eom->num_dofs * eom->num_dofs;
    memcpy(out_matrix, eom->mass_matrix, size * sizeof(vde_real));
}

/**
 * Get the force vector
 * 
 * Input:
 *   - eom: Equations of motion (must be non-NULL)
 *   - out_forces: Output buffer (must have space for num_dofs elements)
 * 
 * Output:
 *   - Copies force vector to out_forces
 */
void eom_get_force_vector(const EquationsOfMotion* eom, vde_real* out_forces) {
    if (!eom || !out_forces) return;
    
    memcpy(out_forces, eom->force_vector, eom->num_dofs * sizeof(vde_real));
}

//-------------------------
// Solving
//-------------------------

/**
 * Solve for accelerations: M * q_ddot = Q
 * 
 * Solves the linear system to find accelerations given mass matrix
 * and force vector. For 6DOF rigid body, this gives linear and
 * angular accelerations.
 * 
 * Guiggiani Reference: Section 3.6
 * 
 * Input:
 *   - eom: Equations of motion (must be non-NULL, must have been built)
 *   - out_accelerations: Output buffer (must have space for num_dofs elements)
 * 
 * Output:
 *   - out_accelerations: Computed accelerations (linear and angular)
 * 
 * Functionality:
 *   1. Solve M * q_ddot = Q using appropriate method
 *   2. For diagonal M (unconstrained), this is simple division
 *   3. For constrained systems, may need iterative solver
 */
void eom_solve_accelerations(const EquationsOfMotion* eom, vde_real* out_accelerations) {
    if (!eom || !out_accelerations) return;
    
    // TODO: Implement acceleration solve
    // For diagonal mass matrix: q_ddot[i] = Q[i] / M[i][i]
    // For general system: Use Gaussian elimination or iterative solver
    
    memcpy(out_accelerations, eom->accelerations, eom->num_dofs * sizeof(vde_real));
}
