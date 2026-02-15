#include "core/integrator/semi_implicit_euler.h"
#include <stdlib.h>
#include <string.h>

//-------------------------
// Internal State
//-------------------------

struct SemiImplicitEuler {
    vde_real* derivatives;  // Temporary storage for derivatives
    int capacity;           // Allocated capacity
};

//-------------------------
// Lifecycle
//-------------------------

/**
 * Create a new semi-implicit Euler integrator
 * 
 * Semi-implicit Euler (also called symplectic Euler) is a first-order
 * integrator that updates velocity first, then uses the new velocity
 * to update position. This preserves energy better than explicit Euler.
 * 
 * Algorithm:
 *   v_new = v_old + a * dt
 *   x_new = x_old + v_new * dt  (uses updated velocity)
 * 
 * Output:
 *   - Returns pointer to newly allocated SemiImplicitEuler, or NULL on failure
 */
SemiImplicitEuler* semi_implicit_euler_create(void) {
    SemiImplicitEuler* integrator = (SemiImplicitEuler*)malloc(sizeof(SemiImplicitEuler));
    if (!integrator) return NULL;
    
    integrator->derivatives = NULL;
    integrator->capacity = 0;
    return integrator;
}

/**
 * Destroy a semi-implicit Euler integrator and free its resources
 * 
 * Input:
 *   - integrator: Integrator to destroy (can be NULL)
 */
void semi_implicit_euler_destroy(SemiImplicitEuler* integrator) {
    if (!integrator) return;
    
    if (integrator->derivatives) {
        free(integrator->derivatives);
    }
    free(integrator);
}

//-------------------------
// Integration
//-------------------------

/**
 * Perform a semi-implicit Euler integration step
 * 
 * Updates velocities first, then positions using the new velocities.
 * This ordering gives better energy conservation than explicit Euler.
 * 
 * Input:
 *   - integrator: Integrator instance (must be non-NULL)
 *   - state: Current state vector (in/out, must be non-NULL)
 *   - n: Size of state vector (must be > 0)
 *   - dt: Time step (must be > 0)
 *   - deriv_func: Function to compute derivatives (must be non-NULL)
 *   - user_data: User data passed to derivative function (can be NULL)
 * 
 * Output:
 *   - state: Updated state after integration step
 * 
 * Functionality:
 *   1. Compute derivatives at current state
 *   2. Update velocity components: v_new = v + dv/dt * dt
 *   3. Update position components: x_new = x + v_new * dt
 */
void semi_implicit_euler_step(
    SemiImplicitEuler* integrator,
    vde_real* state,
    int n,
    vde_real dt,
    DerivativeFunc deriv_func,
    void* user_data
) {
    if (!integrator || !state || !deriv_func|| n <= 0 || dt <= (vde_real)0.0) {
        return;
    }
    
    // Ensure we have enough storage for derivatives
    if (integrator->capacity < n) {
        vde_real* new_derivatives = (vde_real*)realloc(integrator->derivatives, n * sizeof(vde_real));
        if (!new_derivatives) return;
        integrator->derivatives = new_derivatives;
        integrator->capacity = n;
    }
    
    // TODO: Implement semi-implicit Euler integration
    // 1. Compute derivatives
    // deriv_func(state, integrator->derivatives, user_data);
    // 2. Update velocities first
    // 3. Update positions using new velocities
}
