#pragma once
// Vehicle Dynamics Engine - Vehicle Parameters
// Guiggiani: Complete vehicle parameter set for all subsystems.
//
// DESIGN PRINCIPLE (Zero Runtime Overhead):
//   Parameters are loaded ONCE at initialisation and applied to component
//   structs via vehicle_params_apply_to_*(). During the simulation hot-loop,
//   components operate from their own local copies — the parameter system
//   contributes ZERO overhead after init.
//
// USAGE PATTERN:
//   // Option A: Use compile-time defaults (no allocation at all)
//   const VehicleParameters* p = vehicle_params_get_defaults();
//
//   // Option B: Load from file (one-time malloc + parse, then free)
//   VehicleParameters* p = vehicle_params_create();
//   vehicle_params_load_from_file(p, "data/vehicles/TBReCar.txt");
//
//   // Apply to components once — copies values, no further dependency on p
//   vehicle_params_apply_to_suspension(p, suspension);
//   vehicle_params_apply_to_brakes(p, brakes);
//   // ... etc.
//   vehicle_params_destroy(p);   // safe; simulation continues unaffected

#include "core/math/math_base.h"
#include "core/math/mat3.h"
#include "core/math/vec3.h"

//-------------------------
// Forward Declarations
// (component types remain opaque — no circular includes)
//-------------------------

typedef struct Suspension           Suspension;
typedef struct Wheel                Wheel;
typedef struct Brakes               Brakes;
typedef struct Steering             Steering;
typedef struct Driveline            Driveline;
typedef struct Aerodynamics         Aerodynamics;
typedef struct SprungMass           SprungMass;
typedef struct MagicFormulaTireModel MagicFormulaTireModel;

//-------------------------
// Drive Configuration Enum
//-------------------------

typedef enum DriveConfig {
    DRIVE_FWD = 0,   // Front-wheel drive
    DRIVE_RWD = 1,   // Rear-wheel drive (TBRe default)
    DRIVE_AWD = 2    // All-wheel drive
} DriveConfig;

//-------------------------
// Sub-Parameter Structs
// Each group maps to one subsystem, laid out contiguously for
// cache-efficient memcpy during component initialisation.
//-------------------------

// --- Mass & Inertia (Guiggiani Sec. 3.10, Ch. 9) ---
typedef struct VehicleMassParams {
    vde_real total_mass;       // kg     — vehicle + driver
    vde_real sprung_mass;      // kg     — chassis portion (on springs)
    vde_real unsprung_mass[4]; // kg     — per corner [FL, FR, RL, RR]
    vde_real inertia_roll;     // Ixx    kg·m² (about longitudinal body axis)
    vde_real inertia_pitch;    // Iyy    kg·m² (about lateral body axis)
    vde_real inertia_yaw;      // Izz    kg·m² (about vertical body axis)
    vde_real cg_height;        // m      — CG above ground plane
} VehicleMassParams;

// --- Chassis Geometry (Guiggiani Sec. 3.2) ---
typedef struct VehicleGeometryParams {
    vde_real wheelbase;         // m — front-to-rear axle centre distance
    vde_real track_front;       // m — front axle centre-to-centre (tyres)
    vde_real track_rear;        // m — rear  axle centre-to-centre (tyres)
    vde_real cg_to_front_axle;  // m — (a) longitudinal: CG → front axle
    vde_real cg_to_rear_axle;   // m — (c) longitudinal: CG → rear  axle
} VehicleGeometryParams;

// --- Suspension — per corner [FL, FR, RL, RR]  (Guiggiani Sec. 3.8, Ch. 8) ---
typedef struct VehicleSuspensionParams {
    vde_real spring_rate[4];        // N/m   — linear spring stiffness
    vde_real damper_coeff[4];       // N·s/m — linear damping coefficient
    vde_real rest_length[4];        // m     — spring natural (unloaded) length
    vde_real roll_stiffness_front;  // N·m/rad — front anti-roll bar
    vde_real roll_stiffness_rear;   // N·m/rad — rear  anti-roll bar
} VehicleSuspensionParams;

// --- Wheel & Tyre Geometry (Guiggiani Sec. 2.7, 2.11) ---
typedef struct VehicleWheelParams {
    vde_real radius;            // m      — effective rolling radius
    vde_real width;             // m      — tyre section width
    vde_real inertia;           // kg·m²  — rotational inertia per wheel
    vde_real rolling_resistance;// —      — Crr coefficient (dimensionless)
} VehicleWheelParams;

// --- Aerodynamics (Guiggiani Sec. 7.6) ---
typedef struct VehicleAeroParams {
    vde_real drag_coeff;    // Cd         — dimensionless drag coefficient
    vde_real lift_coeff;    // Cl         — negative for downforce
    vde_real frontal_area;  // m²         — reference frontal area
    vde_real aero_balance;  // —          — front downforce fraction [0, 1]
    vde_real air_density;   // kg/m³      — 1.225 at sea level / 20 °C
} VehicleAeroParams;

// --- Brake System (Guiggiani Ch. 4) ---
typedef struct VehicleBrakeParams {
    vde_real max_torque_front;  // N·m — maximum per front wheel
    vde_real max_torque_rear;   // N·m — maximum per rear  wheel
    vde_real brake_balance;     // —   — front fraction [0, 1] (Guiggiani Sec. 4.6)
} VehicleBrakeParams;

// --- Steering (Guiggiani Sec. 3.2.3) ---
typedef struct VehicleSteeringParams {
    vde_real ratio;             // — — steering-wheel angle / road-wheel angle
    vde_real max_angle;         // rad — max road-wheel steer angle (lock-to-lock half)
    vde_real ackermann_factor;  // — — 0 = parallel steer, 1 = full Ackermann
} VehicleSteeringParams;

// --- Driveline (Guiggiani Sec. 3.11.4, 6.1, 7.1) ---
typedef struct VehicleDrivelineParams {
    DriveConfig  drive_config;           // FWD / RWD / AWD
    vde_real     engine_max_torque;      // N·m — at crankshaft (peak)
    vde_real     gear_ratios[7];         // — — [0]=1st … [5]=6th, [6]=reverse
    vde_real     final_drive_ratio;      // — — chain or diff output ratio
    vde_real     drivetrain_efficiency;  // — — [0, 1] transmission losses
} VehicleDrivelineParams;

// --- Pacejka Magic Formula Tyre Coefficients (Guiggiani Sec. 2.10) ---
// Mirrors MagicFormulaParams for a direct field-copy in apply_to_tire_model().
// All four tyres share one set; extend to per-axle later if needed.
typedef struct VehicleTireModelParams {
    // Longitudinal force coefficients (b-series, simplified MF5.2)
    vde_real b0;   // Cx: shape factor                       (~1.65)
    vde_real b1;   // Dx load variation   [N/N²]            (degradation)
    vde_real b2;   // Dx base             [—]                (≈ peak μx)
    vde_real b3;   // Kx quadratic Fz term
    vde_real b4;   // Kx linear Fz term   (slip stiffness BCD)
    vde_real b5;   // Kx exponential load decay
    vde_real b6;   // Ex quadratic Fz term
    vde_real b7;   // Ex linear Fz term
    vde_real b8;   // Ex base curvature factor               (~-0.5)
    vde_real b9;   // Horizontal shift, linear Fz
    vde_real b10;  // Vertical shift

    // Lateral force coefficients (a-series)
    vde_real a0;   // Cy: shape factor                       (~1.30)
    vde_real a1;   // Dy load variation   [N/N²]
    vde_real a2;   // Dy base             [—]                (≈ peak μy)
    vde_real a3;   // Ky cornering stiffness scaling factor
    vde_real a4;   // Ky load-curve shape (controls saturation)
    vde_real a5;   // Ky reference load   [N]
    vde_real a6;   // Ey linear Fz term
    vde_real a7;   // Ey base curvature factor
    vde_real a8;   // Camber thrust coefficient

    vde_real vertical_stiffness; // N/m — tyre carcass vertical stiffness
} VehicleTireModelParams;

//-------------------------
// Unified Parameter Struct
// All subsystems in one contiguous block — sized for a single memcpy.
//-------------------------

typedef struct VehicleParameters {
    VehicleMassParams       mass;
    VehicleGeometryParams   geometry;
    VehicleSuspensionParams suspension;
    VehicleWheelParams      wheel;
    VehicleAeroParams       aero;
    VehicleBrakeParams      brakes;
    VehicleSteeringParams   steering;
    VehicleDrivelineParams  driveline;
    VehicleTireModelParams  tire_model;

    // Metadata — not consumed by simulation
    char name[64];
    char version[16];
} VehicleParameters;

//-------------------------
// Lifecycle
//-------------------------

/**
 * Return a pointer to the compile-time TBRe FSAE default parameters.
 * Zero allocation. Safe to call from any thread at any time.
 * Do NOT write through this pointer.
 */
VDE_API const VehicleParameters* vehicle_params_get_defaults(void);

/**
 * Allocate a mutable copy initialised from the TBRe defaults.
 * Returns NULL on allocation failure.
 * Caller must call vehicle_params_destroy() when finished.
 */
VDE_API VehicleParameters* vehicle_params_create(void);

/**
 * Free a VehicleParameters allocated by vehicle_params_create().
 * Safe to call with NULL.
 */
VDE_API void vehicle_params_destroy(VehicleParameters* params);

//-------------------------
// File I/O
//-------------------------

/**
 * Load parameters from an INI-style config file, overriding fields in
 * `params`.  Fields absent from the file retain their current values,
 * so calling vehicle_params_create() first gives defaults + overrides.
 *
 * Returns  0 on success,
 *         -1 if the file cannot be opened,
 *         +N (N > 0) = number of unrecognised keys (non-fatal, simulation
 *            can still proceed with the values that were parsed).
 */
VDE_API int vehicle_params_load_from_file(VehicleParameters* params,
                                          const char* filename);

/**
 * Write all parameters to an INI-style file.
 * Returns 0 on success, -1 on failure.
 */
VDE_API int vehicle_params_save_to_file(const VehicleParameters* params,
                                        const char* filename);

//-------------------------
// Apply to Components
// Called ONCE at initialisation.  After this point the VehicleParameters
// struct is no longer needed for simulation — each component owns its copy.
//-------------------------

/** Copy suspension spring/damper/geometry into all four corners. */
VDE_API void vehicle_params_apply_to_suspension(const VehicleParameters* p,
                                                Suspension* s);

/** Copy wheel radius, inertia, and rolling resistance to all four wheels. */
VDE_API void vehicle_params_apply_to_wheels(const VehicleParameters* p,
                                            Wheel* wheels[4]);

/** Copy brake torque limits and front/rear balance. */
VDE_API void vehicle_params_apply_to_brakes(const VehicleParameters* p,
                                            Brakes* b);

/** Copy steering ratio, max angle, Ackermann factor, and chassis geometry. */
VDE_API void vehicle_params_apply_to_steering(const VehicleParameters* p,
                                              Steering* s);

/** Copy driveline configuration, gear ratios, and efficiency. */
VDE_API void vehicle_params_apply_to_driveline(const VehicleParameters* p,
                                               Driveline* d);

/** Copy aerodynamic coefficients, areas, balance, and air density. */
VDE_API void vehicle_params_apply_to_aerodynamics(const VehicleParameters* p,
                                                   Aerodynamics* a);

/** Set sprung-mass inertia tensor from scalar roll/pitch/yaw values. */
VDE_API void vehicle_params_apply_to_sprung_mass(const VehicleParameters* p,
                                                  SprungMass* sm);

/**
 * Populate a MagicFormulaTireModel with the tire model coefficients.
 * Call magic_formula_set_params() internally — caller needs only the model
 * pointer, not MagicFormulaParams directly.
 */
VDE_API void vehicle_params_apply_to_tire_model(const VehicleParameters* p,
                                                MagicFormulaTireModel* mf);