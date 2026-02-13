================================================================================
FULLCARSIM - VEHICLE DYNAMICS ENGINE
Development Plan - Following Guiggiani's "The Science of Vehicle Dynamics"
Last Updated: February 13, 2026
================================================================================

PROJECT PHILOSOPHY:
===================
This vehicle dynamics engine is designed following the rigorous mathematical
foundations and methodology presented in "The Science of Vehicle Dynamics:
Handling, Braking, and Ride of Road and Race Cars" by Massimo Guiggiani.

Guiggiani's Pedagogical Approach:
----------------------------------
1. **Tires First** (Chapter 2) - Tire mechanics before vehicle model
   - Tire slips (longitudinal σ, lateral α, spin ϕ)
   - Magic Formula for empirical modeling (Section 2.10)
   - Grip forces and tire behavior (Sections 2.6-2.9)

2. **Integrated Vehicle Model** (Chapter 3) - All subsystems together
   - Three-equation structure: Equilibrium + Constitutive + Congruence (Section 3.12)
   - Suspensions integrated from start (Section 3.8)
   - Load transfers as fundamental (Section 3.7)
   - Sprung/unsprung masses (Section 3.10)

3. **Specific Applications** (Chapters 4-7)
   - Braking Performance (Ch. 4): Optimal brake balance, max deceleration
   - Kinematics of Cornering (Ch. 5): Velocity/acceleration centers
   - Handling of Road Cars (Ch. 6): Understeer/oversteer, MAP analysis
   - Handling of Race Cars (Ch. 7): Locked diff, downforce, handling surface

4. **Ride and Road Holding** (Chapter 8)
   - Quarter car model (Section 8.2)
   - Comfort vs. road holding trade-off (Section 8.3)
   - Full vehicle vibration modes (Section 8.5)

5. **Full 3D Dynamics** (Chapter 9)
   - Yaw, pitch, roll (Section 9.2)
   - Vehicle Invariant Point - VIP (Section 9.5.2)
   - 3D equilibrium equations (Section 9.6)

6. **Advanced Tire Theory** (Chapter 10) - Physical brush model
   - Contact patch theory (Sections 10.1-10.2)
   - Adhesion and sliding zones (Section 10.3)
   - Transient behavior (Section 10.8)

Key Implementation Principles:
-------------------------------
- **Rigorous math**: Follow Guiggiani's equations exactly, including sign conventions
- **Physical insight**: Essential physics first, add detail incrementally
- **Three-equation structure**: Separate equilibrium, constitutive, congruence throughout
- **Validation essential**: Compare with Guiggiani's examples at each phase
- **Integrated approach**: Tires, suspensions, chassis work together from the start

CURRENT STRUCTURE (As Implemented):
====================================

FullCarSim/
│
├── include/                      # Public API headers
│   │
│   ├── core/                     # Core engine subsystems
│   │   ├── math/                 # ✅ COMPLETE - Math library (headers + impl)
│   │   │   ├── math_base.h      # Core types, constants, precision config
│   │   │   ├── vec3.h           # 3D vector operations
│   │   │   ├── mat3.h           # 3x3 matrix operations (rotation matrices)
│   │   │   ├── quat.h           # Quaternion operations (SLERP, integration)
│   │   │   └── frames.h         # Rigid body transforms (Guiggiani Ch. 5, 9)
│   │   │
│   │   ├── integrator/           # ✅ API DEFINED - Numerical integration
│   │   │   ├── integrator_base.h    # Base integrator interface
│   │   │   ├── semi_implicit_euler.h # Semi-implicit Euler (stable for mechanics)
│   │   │   └── runge_kutta4.h       # RK4 (higher accuracy)
│   │   │
│   │   ├── physics/              # ✅ API DEFINED - Dynamics solvers
│   │   │   ├── equations_of_motion.h # 6DOF EOM (Guiggiani Ch. 3, 9)
│   │   │   ├── dynamics_solver.h     # Main dynamics solver
│   │   │   └── constraints.h         # Constraint solver (joints, ground contact)
│   │   │
│   │   └── utils/                # ✅ API DEFINED - Utilities
│   │       └── logger.h          # Logging system
│   │
│   ├── vehicle/                  # ✅ API DEFINED - Vehicle components
│   │   ├── vehicle.h            # Main vehicle interface (Guiggiani Ch. 3)
│   │   ├── sprung_mass.h        # Chassis rigid body (Ch. 3, Sec. 3.10)
│   │   ├── unsprung_mass.h      # Unsprung mass dynamics (Ch. 3, Sec. 3.10)
│   │   ├── tire.h               # Tire interface (Guiggiani Ch. 2)
│   │   ├── wheel.h              # Wheel dynamics (spin, slip) (Ch. 2, Sec. 2.11)
│   │   ├── suspension.h         # Suspension forces (Ch. 3, Sec. 3.8; Ch. 8)
│   │   ├── steering.h           # Steering kinematics (Ackermann)
│   │   ├── brakes.h             # Brake system (Guiggiani Ch. 4)
│   │   ├── driveline.h          # Engine, transmission, differential (Ch. 6-7)
│   │   ├── aerodynamics.h       # Aero forces (race cars in Ch. 7)
│   │   └── vehicle_parameters.h # Vehicle parameter management
│   │
│   ├── tire_models/              # ✅ API DEFINED - Tire models
│   │   ├── brush_models.h       # Brush tire model (Guiggiani Ch. 10)
│   │   ├── magic_formula.h      # Pacejka Magic Formula (Ch. 2, Sec. 2.10)
│   │   └── tire_utilities.h     # Slip calculations (Ch. 2, Sec. 2.7)
│   │
│   ├── track/                    # ✅ API DEFINED - Track/environment
│   │   ├── track_surface.h      # Surface properties, friction (Ch. 8)
│   │   ├── track_geometry.h     # Track geometry, curvature (Ch. 5)
│   │   └── friction_map.h       # 2D friction coefficient map
│   │
│   ├── simulation/               # ✅ API DEFINED - Simulation control
│   │   ├── simulation_config.h  # Main simulation API (working impl)
│   │   ├── simulation_loop.h    # Simulation loop structure
│   │   └── telemetry.h          # Data recording and export
│   │
│   └── input/                    # ✅ API DEFINED - Control inputs
│       └── control.h            # Driver input system
│
├── src/                          # Implementation files
│   ├── core/
│   │   ├── math/                 # ✅ COMPLETE - All implemented
│   │   ├── integrator/           # 📝 TODO - Implement integrators
│   │   ├── physics/              # 📝 TODO - Implement solvers
│   │   └── utils/                # 📝 TODO - Implement logger
│   ├── vehicle/                  # 🚧 PARTIAL - vehicle.c has basic 2D impl
│   ├── tire_models/              # 📝 TODO - Implement tire models
│   ├── track/                    # 📝 TODO - Implement track system
│   ├── simulation/               # 🚧 PARTIAL - sim.c has basic loop
│   ├── input/                    # 📝 TODO - Implement control system
│   └── api.c                     # API re-export
│
├── unityInterface/               # Unity/External interface
│   ├── include/sim_api.h        # Simplified Unity API
│   ├── src/
│   │   ├── sim_api.c
│   │   └── debug_main.c         # Standalone test executable
│   └── build/                    # CMake build files
│
├── data/                         # Runtime data files
│   ├── configs/                  # Simulation configurations
│   ├── tracks/                   # Track definitions
│   └── vehicles/                 # Vehicle parameter files (Guiggiani format)
│
├── CODING_STANDARDS.md           # 📄 Coding standards - READ FIRST
├── AI_CONTEXT.md                 # 📄 For AI assistants - Architecture overview
└── README.md                     # Project overview


IMPLEMENTATION ROADMAP - Following Guiggiani's Methodology:
============================================================

STATUS: All header APIs are defined. Implementation follows Guiggiani's pedagogical order.

Reference: "The Science of Vehicle Dynamics: Handling, Braking, and Ride of Road and Race Cars"
               by Massimo Guiggiani

Phase 1: FOUNDATIONAL MATH & INTEGRATION [CRITICAL FOUNDATION]
---------------------------------------------------------------
Priority: IMMEDIATE - Required before all else

Component Status:
  ✅ core/math/vec3.h, mat3.h, quat.h   - 3D math defined & implemented
  ✅ core/math/frames.h                 - Reference frames defined & implemented
  ✅ core/integrator/*.h                - Integrators defined

Implementation Tasks:
  [ ] Verify math library matches vehicle dynamics conventions:
      - Right-hand coordinate system
      - Proper quaternion normalization
      - Frame transformations for body-fixed vs. inertial
  [ ] Implement numerical integrators:
      - Semi-implicit Euler (stable for mechanical systems)
      - RK4 (higher accuracy)
      - Special handling for quaternion integration (normalize after step)
  [ ] Test with simple rigid body motion

Files to Implement:
  - src/core/integrator/integrator_base.c
  - src/core/integrator/semi_implicit_euler.c
  - src/core/integrator/runge_kutta4.c

Guiggiani References:
  - Chapter 5, Section 5.1: Planar kinematics (velocity/acceleration fields)
  - Chapter 9: 3D vehicle dynamics (orientation, angular velocity)


Phase 2: TIRE MECHANICS [CRITICAL - START HERE PER GUIGGIANI]
--------------------------------------------------------------
Reference: Guiggiani Chapter 2 - "Mechanics of the Wheel with Tire"
           Guiggiani Chapter 10 - "Tire Models" (detailed brush model)

Priority: IMMEDIATE - Guiggiani introduces tires FIRST before vehicle model

Component Status:
  ✅ vehicle/tire.h                - Tire interface defined
  ✅ tire_models/brush_models.h    - Brush model defined
  ✅ tire_models/magic_formula.h   - Pacejka model defined
  ✅ tire_models/tire_utilities.h  - Utilities defined

Implementation Tasks:
  [ ] Implement tire slip definitions (Section 2.7):
      - Rolling velocity (Section 2.7.1)
      - Longitudinal slip σ (Section 2.7.2)
      - Slip angle α (Section 2.7.3)
      - Sign conventions (critical!)
  [ ] Implement Magic Formula for initial testing (Section 2.10):
      - Pure longitudinal slip Fx(σ)
      - Pure lateral slip Fy(α)
      - Use simplified MF52 coefficients
      - Combined slip (basic approximation)
  [ ] Implement tire utilities:
      - Contact patch position
      - Tire coordinate frame
      - Slip calculations from wheel kinematics
  [ ] (LATER) Implement full brush model (Chapter 10):
      - Contact patch geometry (Sections 10.1-10.2)
      - Adhesion and sliding zones (Section 10.3)
      - Steady-state behavior (Sections 10.4-10.7)
      - Transient behavior (Section 10.8)

Files to Implement:
  - src/vehicle/tire.c
  - src/tire_models/magic_formula.c (START HERE)
  - src/tire_models/tire_utilities.c
  - src/tire_models/brush_models.c (LATER - very complex)

Guiggiani Key Concepts:
  - Section 2.2-2.6: Tire as a component, rim motion, contact patch
  - Section 2.7: Tire slips (CRITICAL definitions)
  - Section 2.8: Grip forces vs. tire slips
  - Section 2.10: Magic Formula (empirical model for quick implementation)
  - Chapter 10: Theoretical brush model (for later accuracy)


Phase 3: VEHICLE MODEL FOR HANDLING [HIGH PRIORITY]
----------------------------------------------------
Reference: Guiggiani Chapter 3 - "Vehicle Model for Handling and Performance"

Priority: HIGH - Core vehicle model with suspensions integrated

Component Status:
  ✅ vehicle/sprung_mass.h        - Chassis (body) defined
  ✅ vehicle/unsprung_mass.h      - Wheel assemblies defined
  ✅ vehicle/suspension.h         - Suspension defined
  ✅ vehicle/wheel.h              - Wheel defined

Implementation Tasks:
  [ ] Implement vehicle congruence (kinematic) equations (Section 3.2):
      - Section 3.2.1: Velocities (longitudinal, lateral, yaw rate)
      - Section 3.2.4: Fundamental ratios (speed, curvature)
      - Section 3.2.7: Tire kinematics (compute tire slips from vehicle motion)
  [ ] Implement vehicle equilibrium equations (Sections 3.4-3.6):
      - Forces on vehicle: weight, aero, tire forces (Section 3.5)
      - Longitudinal, lateral, and yaw equations
      - Vertical load calculations
  [ ] Implement load transfers (Section 3.7):
      - Section 3.7.1: Longitudinal load transfer
      - Section 3.7.2: Lateral load transfers
      - Section 3.7.3: Vertical load on each tire
  [ ] Implement suspension (Section 3.8):
      - Section 3.8.5: Roll and vertical stiffnesses
      - Section 3.8.8: No-roll centers and no-roll axis
      - Section 3.8.11: Roll angle and lateral load transfers
      - Section 3.8.12: Explicit lateral load transfer expressions
  [ ] Implement sprung/unsprung masses (Section 3.10)

Files to Implement:
  - src/vehicle/sprung_mass.c
  - src/vehicle/unsprung_mass.c
  - src/vehicle/suspension.c
  - src/vehicle/wheel.c
  - src/core/physics/equations_of_motion.c
  - src/core/physics/dynamics_solver.c

Guiggiani Key Concepts:
  - Section 3.11: Complete vehicle model structure
  - Section 3.12: Model architecture (congruence + constitutive + equilibrium)


Phase 4: BRAKING PERFORMANCE [MEDIUM PRIORITY]
-----------------------------------------------
Reference: Guiggiani Chapter 4 - "Braking Performance"

Priority: MEDIUM - Specific application of vehicle model

Component Status:
  ✅ vehicle/brakes.h - Brake system defined

Implementation Tasks:
  [ ] Implement brake system (Chapter 4):
      - Section 4.4: Longitudinal load transfer under braking
      - Section 4.5: Maximum deceleration
      - Section 4.6: Brake balance (front/rear distribution)
      - Section 4.7: All possible braking combinations
      - Section 4.11: Formula car braking (with downforce)

Files to Implement:
  - src/vehicle/brakes.c

Guiggiani Key Concepts:
  - Section 4.6: Optimal brake balance for maximum deceleration


Phase 5: VEHICLE SUBSYSTEMS [MEDIUM PRIORITY]
----------------------------------------------
Reference: Various sections in Guiggiani Chapter 3, 6, 7

Priority: MEDIUM - Complete realism

Component Status:
  ✅ vehicle/steering.h      - Steering defined
  ✅ vehicle/driveline.h     - Driveline defined
  ✅ vehicle/aerodynamics.h  - Aerodynamics defined

Implementation Tasks:
  [ ] Steering system:
      - Ackermann geometry
      - Steering angles for each wheel
      - Toe and camber effects
  [ ] Driveline:
      - Section 6.1: Open differential (Chapter 6)
      - Section 7.1: Locked/limited slip differential (Chapter 7)
      - Torque distribution to wheels
      - Engine and transmission (simplified)
  [ ] Aerodynamics:
      - Drag, downforce, side force
      - Aerodynamic balance
      - Speed-dependent effects
      - Critical for race cars (Chapter 7)

Files to Implement:
  - src/vehicle/steering.c
  - src/vehicle/driveline.c
  - src/vehicle/aerodynamics.c

Guiggiani Key Concepts:
  - Section 6.1: Open differential for road cars
  - Section 7.1: Locked differential for race cars
  - Section 7.6: Formula car aerodynamics


Phase 6: HANDLING ANALYSIS [MEDIUM PRIORITY]
---------------------------------------------
Reference: Guiggiani Chapter 6 - "Handling of Road Cars"
           Guiggiani Chapter 7 - "Handling of Race Cars"

Priority: MEDIUM - Analysis tools (not required for basic simulation)

Component Status:
  ✅ simulation/simulation_loop.h - Main loop defined
  ✅ simulation/telemetry.h       - Data recording defined

Implementation Tasks:
  [ ] Implement steady-state handling analysis (Chapter 6):
      - Section 6.4: Single track model
      - Section 6.7: Steady-state conditions
      - Section 6.8-6.10: Handling diagrams and MAP (Map of Achievable Performance)
      - Section 6.11: Transient stability analysis
  [ ] Implement race car handling (Chapter 7):
      - Section 7.1: Locked differential effects
      - Section 7.5: Handling surface (3D extension of handling diagram)
      - Section 7.6: Formula car handling with downforce
  [ ] Telemetry for analyzing handling:
      - Record understeer/oversteer characteristics
      - Record lateral acceleration vs. speed
      - Record load transfers

Files to Implement:
  - Analysis utilities (separate from real-time sim)
  - src/simulation/telemetry.c

Guiggiani Key Concepts:
  - Section 6.8: Classical handling diagram
  - Section 6.10: MAP - a revolutionary global approach
  - Section 7.5: 3D handling surface for race cars


Phase 7: RIDE COMFORT AND ROAD HOLDING [MEDIUM PRIORITY]
---------------------------------------------------------
Reference: Guiggiani Chapter 8 - "Ride Comfort and Road Holding"

Priority: MEDIUM - Important for complete simulation

Component Status:
  ✅ vehicle/suspension.h (already defined in Phase 3)

Implementation Tasks:
  [ ] Implement quarter car model (Section 8.2):
      - Sprung and unsprung masses
      - Natural frequencies and modes (Section 8.2.2)
  [ ] Shock absorber tuning (Section 8.3):
      - Comfort optimization (Section 8.3.1)
      - Road holding optimization (Section 8.3.2)
  [ ] Road profile modeling (Section 8.4):
      - Random road profiles
      - PSD (Power Spectral Density)
  [ ] Full vehicle ride analysis (Section 8.5):
      - Bounce, pitch, roll modes
      - Section 8.5.2: Proportional viscous damping
      - Section 8.6: Tuning of suspension stiffnesses

Files to Implement:
  - Extensions to src/vehicle/suspension.c
  - src/track/track_surface.c (road profiles)

Guiggiani Key Concepts:
  - Section 8.2: Quarter car natural frequencies
  - Section 8.3: Optimal damping for comfort vs. road holding trade-off
  - Section 8.5: Full vehicle free vibrations


Phase 8: 3D DYNAMICS WITH ROLL [HIGH PRIORITY FOR REALISM]
-----------------------------------------------------------
Reference: Guiggiani Chapter 9 - "Handling with Roll Motion"

Priority: HIGH - Upgrades from planar to full 3D dynamics

Component Status:
  ✅ core/math/frames.h, quat.h - Already support 3D orientation

Implementation Tasks:
  [ ] Implement 3D vehicle orientation (Section 9.2):
      - Yaw, pitch, roll angles
      - Euler angles or quaternions
  [ ] Implement 3D angular velocity and acceleration (Sections 9.3-9.4):
      - Angular velocity vector
      - Rate of change of angular momentum
  [ ] Implement lateral velocity with roll (Section 9.5):
      - Vehicle Invariant Point (VIP) - Section 9.5.2
      - Track invariant points - Section 9.5.1
  [ ] Implement full 3D vehicle dynamics (Section 9.6):
      - 6DOF equations with roll coupling
      - Section 9.6.2: Rate of change of angular momentum
      - Section 9.6.5: Including unsprung mass effects
  [ ] Handling analysis with roll (Section 9.7):
      - Load transfers with roll (Section 9.7.2)
      - Coupled lateral and roll dynamics

Files to Implement:
  - Upgrade src/vehicle/sprung_mass.c to full 3D
  - Upgrade src/core/physics/equations_of_motion.c to 6DOF

Guiggiani Key Concepts:
  - Section 9.2: Yaw, pitch, roll definitions
  - Section 9.5.2: VIP concept (critical for accurate kinematics)
  - Section 9.6: Complete 3D dynamics


Phase 9: COMPLETE VEHICLE INTEGRATION [HIGH PRIORITY]
------------------------------------------------------
Reference: Guiggiani Chapter 3, Section 3.11-3.12 - Complete model structure

Priority: HIGH - Bring all components together

Component Status:
  ✅ vehicle/vehicle.h              - Main vehicle interface defined
  ✅ simulation/simulation_loop.h   - Simulation loop defined

Implementation Tasks:
  [ ] Assemble complete vehicle system (Section 3.11):
      - Connect all subsystems
      - Section 3.11.1: Equilibrium equations
      - Section 3.11.2: Constitutive (tire) equations
      - Section 3.11.3: Congruence (kinematic) equations
      - Section 3.11.4: Differential mechanism
  [ ] Implement model structure (Section 3.12):
      - Proper separation of concerns
      - Force accumulation
      - Moment accumulation
  [ ] Main simulation loop:
      - Initialize vehicle state
      - Compute tire slips (congruence)
      - Compute tire forces (constitutive)
      - Solve dynamics (equilibrium)
      - Integrate state forward in time
      - Update telemetry

Files to Implement:
  - src/vehicle/vehicle.c (expand from basic 2D)
  - src/simulation/simulation_loop.c
  - src/core/physics/dynamics_solver.c

Guiggiani Key Concepts:
  - Section 3.12: Three-equation structure (equilibrium + constitutive + congruence)
  - This is the heart of Guiggiani's methodology


Phase 10: TRACK SYSTEM [MEDIUM PRIORITY]
-----------------------------------------
Reference: Guiggiani Chapter 5, Section 5.2 - "The Kinematics of Cornering"
           (Track geometry integrated with vehicle kinematics)

Priority: MEDIUM - Required for realistic scenarios

Component Status:
  ✅ track/track_surface.h   - Surface properties defined
  ✅ track/track_geometry.h  - Track geometry defined
  ✅ track/friction_map.h    - Friction mapping defined

Implementation Tasks:
  [ ] Track geometry:
      - Centerline definition
      - Track curvature (relates to Chapter 5 cornering kinematics)
      - Banking angles
      - Elevation profiles
  [ ] Surface properties:
      - Friction coefficient (dry/wet)
      - Surface roughness
      - Grip variation
  [ ] Ground contact:
      - Normal force calculation
      - Contact point determination
      - Surface normal vectors

Files to Implement:
  - src/track/track_surface.c
  - src/track/track_geometry.c
  - src/track/friction_map.c

Guiggiani Key Concepts:
  - Chapter 5: Kinematics of cornering (relates track curvature to vehicle motion)


Phase 11: VALIDATION & TESTING [CRITICAL]
------------------------------------------
Reference: Validation against Guiggiani's examples throughout the book

Priority: HIGH - Ensure physical correctness

Validation Tasks:
  [ ] Unit tests for each component
  [ ] Integration tests for subsystems
  [ ] Validate against Guiggiani examples:
      - Chapter 4: Braking performance examples
      - Chapter 6: Steady-state handling examples
      - Chapter 7: Race car handling
      - Chapter 8: Ride analysis
  [ ] Compare with known vehicle data
  [ ] Stability analysis
  [ ] Step response tests

Files to Create:
  - tests/ directory with validation suite


Phase 12: PARAMETER MANAGEMENT [MEDIUM PRIORITY]
-------------------------------------------------
Reference: Vehicle parameters as used throughout Guiggiani

Priority: MEDIUM - User-friendly parameter handling

Component Status:
  ✅ vehicle/vehicle_parameters.h - Parameter management defined

Implementation Tasks:
  [ ] Parameter file format
  [ ] Load/save vehicle parameters
  [ ] Validate parameter ranges
  [ ] Provide example vehicles

Files to Implement:
  - src/vehicle/vehicle_parameters.c
  - data/vehicles/*.txt (parameter files)


Phase 13: CONTROL & INPUT [LOW PRIORITY]
-----------------------------------------
Reference: Not covered in Guiggiani (external to vehicle dynamics theory)

Priority: LOW - Quality of life, not core physics

Component Status:
  ✅ input/control.h - Control interface defined

Implementation Tasks:
  [ ] Driver input processing
  [ ] Input filtering and smoothing
  [ ] Control recording/playback

Files to Implement:
  - src/input/control.c


Phase 14: UTILITIES [LOW PRIORITY]
-----------------------------------
Reference: N/A - Supporting infrastructure

Priority: LOW - Nice to have

Component Status:
  ✅ core/utils/logger.h  - Logger defined

Implementation Tasks:
  [ ] Logging system
  [ ] Performance profiling

Files to Implement:
  - src/core/utils/logger.c


================================================================================
BOOK REFERENCE - COMPLETE TABLE OF CONTENTS
================================================================================

"The Science of Vehicle Dynamics: Handling, Braking, and Ride of Road and Race Cars"
by Massimo Guiggiani

1 Introduction ............................... 1
  1.1 Vehicle Definition ......................... 2
  1.2 Vehicle Basic Scheme ....................... 3
  References ................................ 6

2 Mechanics of the Wheel with Tire ................... 7
  2.1 The Tire as a Vehicle Component ................. 8
  2.2 Rim Position and Motion ..................... 9
  2.3 Carcass Features ......................... 12
  2.4 Contact Patch ........................... 13
  2.5 Footprint Force .......................... 14
      2.5.1 Perfectly Flat Road Surface ............... 16
  2.6 Tire Global Mechanical Behavior ................. 17
      2.6.1 Tire Transient Behavior ................. 17
      2.6.2 Tire Steady-State Behavior ............... 18
      2.6.3 Rolling Resistance .................... 20
      2.6.4 Speed Independence (Almost) .............. 21
      2.6.5 Pure Rolling (not Free Rolling) ............. 21
  2.7 Tire Slips .............................. 26
      2.7.1 Rolling Velocity ..................... 27
      2.7.2 Definition of Tire Slips ................. 27
      2.7.3 Slip Angle ........................ 30
  2.8 Grip Forces and Tire Slips .................... 31
  2.9 Tire Testing ............................ 33
      2.9.1 Pure Longitudinal Slip ................. 34
      2.9.2 Pure Lateral Slip .................... 35
  2.10 Magic Formula .......................... 38
  2.11 Mechanics of Wheels with Tire .................. 39
  2.12 Summary .............................. 43
  2.13 List of Some Relevant Concepts ................. 44
  References ................................ 44

3 Vehicle Model for Handling and Performance ............ 47
  3.1 Mathematical Framework ..................... 48
  3.2 Vehicle Congruence (Kinematic) Equations ........... 48
      3.2.1 Velocities ......................... 48
      3.2.2 Yaw Angle and Trajectory ................ 49
      3.2.3 Velocity Center ...................... 51
      3.2.4 Fundamental Ratios .................... 52
      3.2.5 Accelerations and Radii of Curvature ......... 53
      3.2.6 Acceleration Center ................... 54
      3.2.7 Tire Kinematics (Tire Slips) ............... 56
  3.3 Vehicle Constitutive (Tire) Equations ............. 58
  3.4 Vehicle Equilibrium Equations .................. 59
  3.5 Forces Acting on the Vehicle ................... 59
      3.5.1 Weight ........................... 60
      3.5.2 Aerodynamic Force ................... 60
      3.5.3 Road-Tire Friction Forces ................ 61
      3.5.4 Road-Tire Vertical Forces ................ 63
  3.6 Vehicle Equilibrium Equations (more Explicit Form) ...... 63
  3.7 Load Transfers .......................... 65
      3.7.1 Longitudinal Load Transfer ............... 65
      3.7.2 Lateral Load Transfers ................. 66
      3.7.3 Vertical Loads on Each Tire .............. 66
  3.8 Suspension First-Order Analysis ................. 67
      3.8.1 Suspension Reference Configuration .......... 67
      3.8.2 Suspension Internal Coordinates ............ 68
      3.8.3 Camber Variation .................... 69
      3.8.4 Vehicle Internal Coordinates .............. 70
      3.8.5 Roll and Vertical Stiffnesses .............. 71
      3.8.6 Suspension Internal Equilibrium ............ 73
      3.8.7 Effects of a Lateral Force ................ 74
      3.8.8 No-roll Centers and No-roll Axis ............ 75
      3.8.9 Forces at the No-roll Centers .............. 77
      3.8.10 Suspension Jacking .................. 78
      3.8.11 Roll Angle and Lateral Load Transfers ........ 79
      3.8.12 Explicit Expressions of Lateral Load Transfers ... 81
      3.8.13 Lateral Load Transfers with Rigid Tires ....... 82
  3.9 Dependent Suspensions ...................... 82
  3.10 Sprung and Unsprung Masses .................. 85
  3.11 Vehicle Model for Handling and Performance ......... 86
      3.11.1 Equilibrium Equations ................. 86
      3.11.2 Constitutive (Tire) Equations ............. 88
      3.11.3 Congruence (Kinematic) Equations .......... 88
      3.11.4 Principles of Any Differential Mechanism ...... 90
  3.12 The Structure of This Vehicle Model .............. 94
  3.13 Three-Axle Vehicles ....................... 95
  3.14 Summary .............................. 97
  3.15 List of Some Relevant Concepts ................. 97
  References ................................ 98

4 Braking Performance ......................... 99
  4.1 Pure Braking ............................ 99
  4.2 Vehicle Model for Braking Performance ............. 100
  4.3 Equilibrium Equations ...................... 101
  4.4 Longitudinal Load Transfer ................... 101
  4.5 Maximum Deceleration ...................... 102
  4.6 Brake Balance ........................... 103
  4.7 All Possible Braking Combinations ............... 103
  4.8 Changing the Grip ......................... 105
  4.9 Changing the Weight Distribution ................ 106
  4.10 A Numerical Example ...................... 106
  4.11 Braking Performance of Formula Cars ............. 107
      4.11.1 Equilibrium Equations ................. 107
      4.11.2 Longitudinal Load Transfer .............. 108
      4.11.3 Maximum Deceleration ................. 108
      4.11.4 Braking Balance .................... 109
      4.11.5 Typical Formula 1 Braking Performance ....... 109
  4.12 Summary .............................. 109
  4.13 List of Some Relevant Concepts ................. 110
  References ................................ 111

5 The Kinematics of Cornering ..................... 113
  5.1 Planar Kinematics of a Rigid Body ................ 113
      5.1.1 Velocity Field and Velocity Center ........... 113
      5.1.2 Acceleration Field, Inflection Circle and 
            Acceleration Center ................... 115
  5.2 The Kinematics of a Turning Vehicle .............. 119
      5.2.1 Fixed and Moving Centrodes of a Turning Vehicle ... 119
      5.2.2 Inflection Circle ..................... 123
      5.2.3 Variable Curvatures ................... 126
  References ................................ 130

6 Handling of Road Cars ......................... 131
  6.1 Open Differential ......................... 131
  6.2 Fundamental Equations of Vehicle Handling .......... 132
  6.3 Double Track Model ....................... 136
  6.4 Single Track Model ........................ 137
      6.4.1 Governing Equations of the Single Track Model .... 138
      6.4.2 Axle Characteristics ................... 140
  6.5 Alternative State Variables .................... 144
      6.5.1 β and ρ as State Variables ............... 145
      6.5.2 β1 and β2 as State Variables .............. 147
      6.5.3 S and R as State Variables ............... 149
  6.6 Inverse Congruence Equations .................. 149
  6.7 Vehicle in Steady-State Conditions ............... 150
      6.7.1 The Role of the Steady-State Lateral Acceleration ... 151
      6.7.2 Steady-State Analysis .................. 153
  6.8 Handling Diagram—The Classical Approach .......... 154
  6.9 Weak Concepts in Classical Vehicle Dynamics ......... 158
      6.9.1 Popular Definitions of Understeer/Oversteer ...... 159
  6.10 Map of Achievable Performance (MAP)—A New Global
       Approach .............................. 159
      6.10.1 MAP Curvature ρ vs Steer Angle δ .......... 161
      6.10.2 MAP: Vehicle Slip Angle β vs Curvature ρ ...... 165
  6.11 Vehicle in Transient Conditions (Stability and Control
       Derivatives) ............................ 169
      6.11.1 Steady-State Conditions (Equilibrium Points) .... 170
      6.11.2 Linearization of the Equations of Motion ....... 171
      6.11.3 Stability ......................... 173
      6.11.4 Forced Oscillations (Driver Action) .......... 173
  6.12 Relationship Between Steady State Data and Transient
       Behavior .............................. 175
  6.13 New Understeer Gradient .................... 179
  6.14 Stability (Again) ......................... 180
  6.15 The Single Track Model Revisited ............... 180
      6.15.1 Different Vehicles with Almost Identical Handling . 184
  6.16 Road Vehicles with Locked or Limited Slip Differential ... 186
  6.17 Linear Single Track Model ................... 186
      6.17.1 Governing Equations .................. 187
      6.17.2 Solution for Constant Forward Speed ......... 188
      6.17.3 Critical Speed ...................... 190
      6.17.4 Transient Vehicle Behavior .............. 191
      6.17.5 Steady-State Behavior: Steering Pad ......... 193
      6.17.6 Lateral Wind Gust ................... 194
      6.17.7 Banked Road ...................... 198
  6.18 Compliant Steering System ................... 198
      6.18.1 Governing Equations .................. 199
      6.18.2 Effects of Compliance ................. 200
  6.19 Summary .............................. 201
  6.20 List of Some Relevant Concepts ................. 201
  References ................................ 201

7 Handling of Race Cars ......................... 203
  7.1 Locked and Limited Slip Differentials ............. 203
  7.2 Fundamental Equations of Race Car Handling ......... 205
  7.3 Double Track Race Car Model .................. 208
  7.4 Tools for Handling Analysis ................... 209
  7.5 The Handling Diagram Becomes the Handling Surface ..... 210
      7.5.1 Handling with Locked Differential (no Wings) .... 210
  7.6 Handling of Formula Cars .................... 221
      7.6.1 Handling Surface .................... 223
      7.6.2 Map of Achievable Performance (MAP) ......... 225
  7.7 Summary .............................. 231
  7.8 List of Some Relevant Concepts ................. 233
  References ................................ 233

8 Ride Comfort and Road Holding ................... 235
  8.1 Vehicle Models for Ride and Road Holding .......... 236
  8.2 Quarter Car Model ........................ 239
      8.2.1 The Inerter as a Spring Softener ............ 243
      8.2.2 Quarter Car Natural Frequencies and Modes ...... 244
  8.3 Shock Absorber Tuning ...................... 247
      8.3.1 Comfort Optimization .................. 247
      8.3.2 Road Holding Optimization ............... 248
      8.3.3 The Inerter as a Tool for Road Holding Tuning .... 251
  8.4 Road Profiles ........................... 252
  8.5 Free Vibrations of Road Cars .................. 254
      8.5.1 Governing Equations .................. 254
      8.5.2 Proportional Viscous Damping ............. 256
      8.5.3 Vehicle with Proportional Viscous Damping ...... 257
  8.6 Tuning of Suspension Stiffnesses ................ 262
      8.6.1 Optimality of Proportional Damping .......... 263
      8.6.2 A Numerical Example .................. 264
  8.7 Non-proportional Damping ................... 265
  8.8 Interconnected Suspensions ................... 265
  8.9 Summary .............................. 268
  8.10 List of Some Relevant Concepts ................. 269
  References ................................ 269

9 Handling with Roll Motion ...................... 271
  9.1 Vehicle Position and Orientation ................ 271
  9.2 Yaw, Pitch and Roll ........................ 272
  9.3 Angular Velocity ......................... 275
  9.4 Angular Acceleration ....................... 277
  9.5 Vehicle Lateral Velocity ..................... 277
      9.5.1 Track Invariant Points .................. 277
      9.5.2 Vehicle Invariant Point (VIP) .............. 279
      9.5.3 Lateral Velocity and Acceleration ........... 281
  9.6 Three-Dimensional Vehicle Dynamics .............. 282
      9.6.1 Velocity and Acceleration of G ............. 282
      9.6.2 Rate of Change of the Angular Momentum ....... 284
      9.6.3 Completing the Torque Equation ............ 285
      9.6.4 Equilibrium Equations .................. 285
      9.6.5 Including the Unsprung Mass .............. 286
  9.7 Handling with Roll Motion .................... 287
      9.7.1 Equilibrium Equations .................. 287
      9.7.2 Load Transfers ...................... 287
      9.7.3 Constitutive (Tire) Equations .............. 288
      9.7.4 Congruence (Kinematic) Equations ........... 288
  9.8 Steady-State and Transient Analysis .............. 289
  9.9 Summary .............................. 289
  9.10 List of Some Relevant Concepts ................. 289
  References ................................ 289

10 Tire Models ............................... 291
  10.1 Brush Model Definition ..................... 291
      10.1.1 Roadway and Rim ................... 292
      10.1.2 Shape of the Contact Patch .............. 292
      10.1.3 Force-Couple Resultant ................ 293
      10.1.4 Position of the Contact Patch ............. 294
      10.1.5 Pressure Distribution ................. 295
      10.1.6 Friction ......................... 297
      10.1.7 Constitutive Relationship ............... 297
      10.1.8 Kinematics ....................... 298
  10.2 General Governing Equations of the Brush Model ....... 300
      10.2.1 Data for Numerical Examples ............. 302
  10.3 Brush Model Steady-State Behavior .............. 302
      10.3.1 Governing Equations .................. 303
      10.3.2 Adhesion and Sliding Zones .............. 303
      10.3.3 Force-Couple Resultant ................ 307
  10.4 Adhesion Everywhere (Linear Behavior) ............ 308
  10.5 Wheel with Pure Translational Slip (σ ≠ 0, ϕ = 0) ..... 312
      10.5.1 Rectangular Contact Patch ............... 317
      10.5.2 Elliptical Contact Patch ................ 325
  10.6 Wheel with Pure Spin Slip (σ = 0, ϕ ≠ 0) .......... 326
  10.7 Wheel with Both Translational and Spin Slips ........ 328
      10.7.1 Rectangular Contact Patch ............... 328
      10.7.2 Elliptical Contact Patch ................ 331
  10.8 Brush Model Transient Behavior ................ 334
      10.8.1 Transient Model with Carcass Compliance only ... 336
      10.8.2 Transient Model with Carcass and Tread Compliance 338
      10.8.3 Numerical Examples .................. 341
  10.9 Summary .............................. 344
  10.10 List of Some Relevant Concepts ................ 344
  References ................................ 345

Index ..................................... 347

================================================================================


IMPLEMENTATION NOTES:
=====================

1. ALL new code MUST follow standards in CODING_STANDARDS.md
2. Use vde_real for all floating-point math
3. Use math/ types (vde_vec3, vde_quat, vde_frame) for spatial math
4. Add defensive programming (null checks, validation)
5. Section code with //------------------------- headers
6. Use #pragma once for all headers
7. Document as you go

BEFORE ADDING NEW MODULES:
- Read CODING_STANDARDS.md
- Read AI_CONTEXT.md
- Check existing module structure for patterns
- Maintain consistency with math/ implementation quality

================================================================================
⚠️  IMPORTANT: Keep this file updated as structure evolves
================================================================================