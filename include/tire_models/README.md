# Tire Models - Constitutive Equations

**"Start with the tires!"** - Guiggiani's Pedagogical Approach

This directory contains tire models - the CONSTITUTIVE equations that relate
tire slips (σ, α, φ) to forces and moments. These are introduced in Chapter 2,
BEFORE the vehicle model, because tires are the critical link between vehicle
and road.

## Guiggiani's Tire Philosophy (Chapter 2)

> "The tire is perhaps the most important component of a road vehicle."
> - Guiggiani, Section 2.1

### Why Tires First?

1. **Grip forces enable all vehicle dynamics** - acceleration, braking, cornering
2. **Highly nonlinear behavior** - must understand before vehicle analysis
3. **Constitutive elements** - forces are functions of slips, not directly controlled
4. **Experimental characterization** - requires testing and modeling

---

## Tire Models in This Directory

### `magic_formula.h` - Empirical Tire Model
**Guiggiani Chapter 2, Section 2.10**

The **Pacejka Magic Formula** is an empirical model that fits experimental data:

```
y(x) = D * sin[C * arctan{B*x - E*(B*x - arctan(B*x))}]
```

**Parameters:**
- B: Stiffness factor
- C: Shape factor
- D: Peak value
- E: Curvature factor
- x: Slip (σ for longitudinal, α for lateral)
- y: Force or moment coefficient

**Advantages:**
- ✅ Excellent fit to experimental data
- ✅ Computationally efficient
- ✅ Industry standard
- ✅ Well-documented parameter sets available

**When to use:**
- Realistic vehicle simulation
- When experimental tire data available
- Performance analysis
- **Default model for the simulation**

---

### `brush_models.h` - Physical Tire Model
**Guiggiani Chapter 10 (Advanced Theory)**

The **brush model** is a physical model based on contact patch mechanics:

**Physical Basis:**
1. Contact patch divided into **adhesion** and **sliding** zones
2. Bristles deform elastically in adhesion zone
3. Bristles slide in sliding zone (Coulomb friction)
4. Integration over contact patch gives forces

**Key Sections:**
- 10.1-10.2: Contact patch geometry
- 10.3: Adhesion and sliding zones
- 10.4-10.5: Pure longitudinal and lateral slip
- 10.6: Combined slip
- 10.8: Transient behavior (relaxation length)

**Advantages:**
- ✅ Physical insight into tire behavior
- ✅ Can predict design changes
- ✅ Handles transient behavior naturally
- ✅ No experimental data required (just physical params)

**Disadvantages:**
- ❌ More complex implementation
- ❌ More computationally expensive
- ❌ Requires careful numerical integration

**When to use:**
- Understanding tire physics
- Design sensitivity analysis
- When tire data not available
- Advanced simulation with transients
- **Implement later** after Magic Formula working

---

### `tire_utilities.h` - Slip Calculations
**Guiggiani Chapter 2, Section 2.7; Chapter 3, Section 3.2.7**

Utility functions for computing tire slips from kinematics (CONGRUENCE equations).

#### Longitudinal Slip Ratio (Section 2.7.2)

```
σ = (ω*re - Vx) / |Vx|
```

where:
- ω = wheel angular velocity (rad/s)
- re = effective rolling radius (m)
- Vx = contact point longitudinal velocity (m/s)

**Sign Convention:**
- σ > 0: wheel faster than vehicle (driving)
- σ < 0: wheel slower than vehicle (braking)
- σ = 0: pure rolling

#### Slip Angle (Section 2.7.3)

```
tan(α) = Vy / |Vx|
```

where:
- Vy = contact point lateral velocity (m/s)
- Vx = contact point longitudinal velocity (m/s)
- α = slip angle (rad)

**Physical Meaning:**
Angle between wheel heading direction and actual velocity direction.

#### Spin Slip (Section 2.7.4)

```
φ = (camber-induced spin) / |Vx|
```

Related to camber angle and tire geometry. Less critical than σ and α.

---

## Tire Testing and Characterization (Section 2.9)

Guiggiani describes experimental tire testing:

### Pure Longitudinal Slip (Section 2.9.1)
- Fix slip angle α = 0
- Vary slip ratio σ
- Measure Fx vs σ at different Fz

### Pure Lateral Slip (Section 2.9.2)
- Fix slip ratio σ = 0 (free rolling)
- Vary slip angle α
- Measure Fy vs α at different Fz

### Combined Slip
- Vary both σ and α
- Measure Fx, Fy
- Much more complex!

---

## Integration with Vehicle Model

Tire models are **CONSTITUTIVE** equations in Guiggiani's three-equation structure:

```
1. CONGRUENCE (vehicle_congruence.h)
   └─> Compute tire slips (σ, α, φ) from vehicle motion

2. CONSTITUTIVE (tire models - THIS DIRECTORY)
   └─> Compute tire forces (Fx, Fy, Fz, Mx, My, Mz) from slips

3. EQUILIBRIUM (vehicle_equilibrium.h)
   └─> Apply tire forces to vehicle dynamics equations
```

---

## Implementation Roadmap

### Status: ✅ All tire model source files complete

Magic Formula and utilities are fully implemented. The brush model uses a
simplified linear+saturation approximation with a clear upgrade path to the
full physical implementation (see `src/tire_models/brush_models.c`).

### Remaining: Full brush model bristle dynamics (Chapter 10)
1. Contact patch geometry and bristle deflection
2. Adhesion/sliding zone computation
3. Force integration over contact patch
4. Transient relaxation length behavior

---

## Usage Example

```c
#include "tire_models/magic_formula.h"
#include "tire_models/tire_utilities.h"
#include "vehicle/tire.h"

// Create tire model
MagicFormulaTireModel* mf = magic_formula_create();
MagicFormulaParams params = {
    // Longitudinal coefficients
    .b0 = 1.5, .b1 = 0, .b2 = 1000, /* ... */
    // Lateral coefficients  
    .a0 = 1.3, .a1 = 0, .a2 = 800, /* ... */
    .vertical_stiffness = 200000
};
magic_formula_set_params(mf, &params);

// Compute tire slips from kinematics (congruence)
vde_real sigma = tire_util_compute_slip_ratio(omega, radius, velocity_x);
vde_real alpha = tire_util_compute_slip_angle(velocity_x, velocity_y);

// Build tire state
TireState state = {
    .normal_load = 3000.0,      // 3000 N vertical load
    .slip_ratio = sigma,
    .slip_angle = alpha,
    .camber_angle = 0.0,
    .angular_velocity = omega
};

// Compute forces (constitutive)
TireForces forces;
magic_formula_compute_forces(mf, &state, &forces);

printf("Fx = %.1f N, Fy = %.1f N, Fz = %.1f N\n",
       forces.force.x, forces.force.y, forces.force.z);
```

---

## Key Formulas and Concepts

### Tire Characteristic (Section 2.8)

Typical tire force vs slip curve:
- **Linear region** (small slips): F ≈ C * slip
- **Peak** (optimal slip): Maximum force
- **Sliding region** (large slips): Force decreases

### Friction Circle/Ellipse

Combined slip limits:
```
sqrt((Fx/Fx_max)² + (Fy/Fy_max)²) ≤ 1
```

Tire can't provide maximum longitudinal AND lateral force simultaneously.

### Relaxation Length (Section 10.8)

Tire forces don't respond instantaneously to slip changes:
```
dF/dx = (F_steady_state - F) / σ_relax
```

where σ_relax is relaxation length (~0.1-0.3 m for passenger tires).

---

## References

### Main Chapters:
- **Chapter 2**: Complete tire mechanics overview - **READ THIS FIRST**
- **Chapter 10**: Advanced brush model theory

### Key Sections:
- 2.7: Tire slip definitions (CRITICAL)
- 2.8: Grip forces and tire slips
- 2.9: Tire testing methods
- 2.10: Magic Formula
- 10.1-10.8: Brush model theory

---

## Validation Data

When implementing, validate against Guiggiani's examples and real tire data:
- Typical peak slip ratio: σ_peak ≈ 0.10-0.15
- Typical peak slip angle: α_peak ≈ 10-15 degrees
- Typical friction coefficient: μ ≈ 0.8-1.2 (dry), 0.3-0.5 (wet)

**Always compare results with physical intuition and experimental data!**
