#include "core/integrator/integrator_base.h"
#include <stdlib.h>

//-------------------------
// Internal State
//-------------------------

struct IntegratorBase {
    vde_real* derivatives;  // Temporary storage for derivatives
    int capacity;           // Allocated capacity
};

//-------------------------
// Lifecycle
//-------------------------

/**
 * Create a new integrator base instance
 * 
 * Output:
 *   - Returns pointer to newly allocated IntegratorBase, or NULL on failure
 */
IntegratorBase* integrator_create(void) {
    IntegratorBase* integrator = (IntegratorBase*)malloc(sizeof(IntegratorBase));
    if (!integrator) return NULL;
    
    integrator->derivatives = NULL;
    integrator->capacity = 0;
    return integrator;
}

/**
 * Destroy an integrator and free its resources
 * 
 * Input:
 *   - integrator: Integrator to destroy (can be NULL)
 */
void integrator_destroy(IntegratorBase* integrator) {
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
 * Perform a single integration step
 * 
 * Advances the state vector forward by dt using the derivative function.
 * This is the base implementation - specific integrators override this.
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
 */
void integrator_step(
    IntegratorBase* integrator,
    vde_real* state,
    int n,
    vde_real dt,
    DerivativeFunc deriv_func,
    void* user_data
) {
    if (!integrator || !state || !deriv_func || n <= 0 || dt <= (vde_real)0.0) {
        return;
    }
    
    // Ensure we have enough storage for derivatives
    if (integrator->capacity < n) {
        vde_real* new_derivatives = (vde_real*)realloc(integrator->derivatives, n * sizeof(vde_real));
        if (!new_derivatives) return;
        integrator->derivatives = new_derivatives;
        integrator->capacity = n;
    }
    
    // Explicit Euler: y_new = y_old + f(y_old) * dt
    // 1. Compute derivatives at current state
    deriv_func(state, integrator->derivatives, user_data);
    
    // 2. Update state
    for (int i = 0; i < n; i++) {
        state[i] += integrator->derivatives[i] * dt;
        
        // Safety check for numerical stability
        if (!vde_isfinite(state[i])) {
            state[i] = (vde_real)0.0;
        }
    }
}
