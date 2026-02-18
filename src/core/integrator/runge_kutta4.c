#include "core/integrator/runge_kutta4.h"
#include <stdlib.h>
#include <string.h>

//-------------------------
// Internal State
//-------------------------

struct RungeKutta4 {
    vde_real* k1;        // First slope
    vde_real* k2;        // Second slope
    vde_real* k3;        // Third slope
    vde_real* k4;        // Fourth slope
    vde_real* temp_state; // Temporary state for intermediate evaluations
    int capacity;        // Allocated capacity
};

//-------------------------
// Lifecycle
//-------------------------

/**
 * Create a new Runge-Kutta 4th order integrator
 * 
 * RK4 is a fourth-order explicit integrator that evaluates the derivative
 * at four points per timestep to achieve high accuracy.
 * 
 * Algorithm:
 *   k1 = f(t, y)
 *   k2 = f(t + dt/2, y + dt*k1/2)
 *   k3 = f(t + dt/2, y + dt*k2/2)
 *   k4 = f(t + dt, y + dt*k3)
 *   y_new = y + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
 * 
 * Output:
 *   - Returns pointer to newly allocated RungeKutta4, or NULL on failure
 */
RungeKutta4* rk4_create(void) {
    RungeKutta4* integrator = (RungeKutta4*)malloc(sizeof(RungeKutta4));
    if (!integrator) return NULL;
    
    integrator->k1 = NULL;
    integrator->k2 = NULL;
    integrator->k3 = NULL;
    integrator->k4 = NULL;
    integrator->temp_state = NULL;
    integrator->capacity = 0;
    return integrator;
}

/**
 * Destroy an RK4 integrator and free its resources
 * 
 * Input:
 *   - integrator: Integrator to destroy (can be NULL)
 */
void rk4_destroy(RungeKutta4* integrator) {
    if (!integrator) return;
    
    if (integrator->k1) free(integrator->k1);
    if (integrator->k2) free(integrator->k2);
    if (integrator->k3) free(integrator->k3);
    if (integrator->k4) free(integrator->k4);
    if (integrator->temp_state) free(integrator->temp_state);
    free(integrator);
}

//-------------------------
// Integration
//-------------------------

/**
 * Perform a Runge-Kutta 4th order integration step
 * 
 * RK4 evaluates the derivative at four points and combines them
 * with specific weights to achieve fourth-order accuracy.
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
 *   1. Compute k1 at current state
 *   2. Compute k2 at midpoint using k1
 *   3. Compute k3 at midpoint using k2
 *   4. Compute k4 at endpoint using k3
 *   5. Combine slopes with weights (1, 2, 2, 1) / 6
 */
void rk4_step(
    RungeKutta4* integrator,
    vde_real* state,
    int n,
    vde_real dt,
    DerivativeFunc deriv_func,
    void* user_data
) {
    if (!integrator || !state || !deriv_func || n <= 0 || dt <= (vde_real)0.0) {
        return;
    }
    
    // Ensure we have enough storage for temporary vectors
    if (integrator->capacity < n) {
        vde_real* new_k1 = (vde_real*)realloc(integrator->k1, n * sizeof(vde_real));
        vde_real* new_k2 = (vde_real*)realloc(integrator->k2, n * sizeof(vde_real));
        vde_real* new_k3 = (vde_real*)realloc(integrator->k3, n * sizeof(vde_real));
        vde_real* new_k4 = (vde_real*)realloc(integrator->k4, n * sizeof(vde_real));
        vde_real* new_temp = (vde_real*)realloc(integrator->temp_state, n * sizeof(vde_real));
        
        if (!new_k1 || !new_k2 || !new_k3 || !new_k4 || !new_temp) return;
        
        integrator->k1 = new_k1;
        integrator->k2 = new_k2;
        integrator->k3 = new_k3;
        integrator->k4 = new_k4;
        integrator->temp_state = new_temp;
        integrator->capacity = n;
    }
    
    // Classic Runge-Kutta 4th order method
    // y_new = y + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    
    // Step 1: Compute k1 = f(t, y)
    deriv_func(state, integrator->k1, user_data);
    
    // Step 2: Compute k2 = f(t + dt/2, y + dt*k1/2)
    vde_real half_dt = dt * (vde_real)0.5;
    for (int i = 0; i < n; i++) {
        integrator->temp_state[i] = state[i] + half_dt * integrator->k1[i];
    }
    deriv_func(integrator->temp_state, integrator->k2, user_data);
    
    // Step 3: Compute k3 = f(t + dt/2, y + dt*k2/2)
    for (int i = 0; i < n; i++) {
        integrator->temp_state[i] = state[i] + half_dt * integrator->k2[i];
    }
    deriv_func(integrator->temp_state, integrator->k3, user_data);
    
    // Step 4: Compute k4 = f(t + dt, y + dt*k3)
    for (int i = 0; i < n; i++) {
        integrator->temp_state[i] = state[i] + dt * integrator->k3[i];
    }
    deriv_func(integrator->temp_state, integrator->k4, user_data);
    
    // Step 5: Combine slopes with weighted average
    // y_new = y + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    vde_real dt_sixth = dt / (vde_real)6.0;
    for (int i = 0; i < n; i++) {
        state[i] += dt_sixth * (integrator->k1[i] + 
                                (vde_real)2.0 * integrator->k2[i] + 
                                (vde_real)2.0 * integrator->k3[i] + 
                                integrator->k4[i]);
        
        // Safety check for numerical stability
        if (!vde_isfinite(state[i])) {
            state[i] = (vde_real)0.0;
        }
    }
}
