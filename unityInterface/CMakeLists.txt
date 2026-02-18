cmake_minimum_required(VERSION 3.15)
project(racing_sim_plugin C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Paths relative to unity-interface/
set(SIM_ROOT ${CMAKE_SOURCE_DIR}/..)

# Include directories
include_directories(
    ${SIM_ROOT}/include
    ${CMAKE_SOURCE_DIR}/include
)

# Core simulation source files (shared by both DLL and debug executable)
set(CORE_SRC
    # Core math library
    ${SIM_ROOT}/src/core/math/vec3.c
    ${SIM_ROOT}/src/core/math/mat3.c
    ${SIM_ROOT}/src/core/math/quat.c
    ${SIM_ROOT}/src/core/math/frames.c
    ${SIM_ROOT}/src/core/math/math_base.c
    
    # Core integrators (Euler, RK4, Semi-implicit)
    ${SIM_ROOT}/src/core/integrator/integrator_base.c
    ${SIM_ROOT}/src/core/integrator/runge_kutta4.c
    ${SIM_ROOT}/src/core/integrator/semi_implicit_euler.c
    
    # Core physics
    ${SIM_ROOT}/src/core/physics/equations_of_motion.c
    ${SIM_ROOT}/src/core/physics/dynamics_solver.c
    ${SIM_ROOT}/src/core/physics/constraints.c
    
    # Core utilities
    ${SIM_ROOT}/src/core/utils/logger.c
    
    # Tire models (Magic Formula, Brush)
    ${SIM_ROOT}/src/tire_models/tire_utilities.c
    ${SIM_ROOT}/src/tire_models/magic_formula.c
    ${SIM_ROOT}/src/tire_models/brush_models.c
    
    # Vehicle components
    ${SIM_ROOT}/src/vehicle/tire.c
    ${SIM_ROOT}/src/vehicle/wheel.c
    ${SIM_ROOT}/src/vehicle/suspension.c
    ${SIM_ROOT}/src/vehicle/brakes.c
    ${SIM_ROOT}/src/vehicle/aerodynamics.c
    ${SIM_ROOT}/src/vehicle/steering.c
    ${SIM_ROOT}/src/vehicle/driveline.c
    ${SIM_ROOT}/src/vehicle/sprung_mass.c
    ${SIM_ROOT}/src/vehicle/unsprung_mass.c
    ${SIM_ROOT}/src/vehicle/vehicle_parameters.c
    
    # Three-equation structure (Guiggiani methodology)
    ${SIM_ROOT}/src/vehicle/vehicle_congruence.c
    ${SIM_ROOT}/src/vehicle/vehicle_constitutive.c
    ${SIM_ROOT}/src/vehicle/vehicle_equilibrium.c
    ${SIM_ROOT}/src/vehicle/vehicle_model.c
    
    # Vehicle integration
    ${SIM_ROOT}/src/vehicle/vehicle.c
    
    # Track model
    ${SIM_ROOT}/src/track/track_geometry.c
    ${SIM_ROOT}/src/track/track_surface.c
    ${SIM_ROOT}/src/track/friction_map.c
    ${SIM_ROOT}/src/track/track.c
    
    # Input handling
    ${SIM_ROOT}/src/input/control.c
    
    # Simulation loop and telemetry
    ${SIM_ROOT}/src/simulation/simulation_loop.c
    ${SIM_ROOT}/src/simulation/telemetry.c
    ${SIM_ROOT}/src/simulation/sim.c
)

# Unity DLL-specific source (API layer)
set(DLL_SRC
    ${SIM_ROOT}/src/unity_api.c
)

# Debug executable source
set(DEBUG_SRC
    ${CMAKE_SOURCE_DIR}/src/debug_main.c
)

# ========================================
# TARGET 1: Unity DLL (shared library)
# ========================================
add_library(racing_sim SHARED ${CORE_SRC} ${DLL_SRC})

# Define BUILDING_DLL when compiling the library
target_compile_definitions(racing_sim PRIVATE BUILDING_DLL _CRT_SECURE_NO_WARNINGS)

# Optional: define platform exports
if (WIN32)
    target_compile_definitions(racing_sim PRIVATE -DWIN32)
endif()

# Set output directory to something predictable (like ../build)
set_target_properties(racing_sim PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${SIM_ROOT}/build
    RUNTIME_OUTPUT_DIRECTORY ${SIM_ROOT}/build
    OUTPUT_NAME "racing_sim"
)

# ========================================
# TARGET 2: Debug Test Executable
# ========================================
add_executable(sim_debug ${CORE_SRC} ${DEBUG_SRC})

# Debug exe statically links simulation code - use STATIC_BUILD
target_compile_definitions(sim_debug PRIVATE STATIC_BUILD _CRT_SECURE_NO_WARNINGS)
if (WIN32)
    target_compile_definitions(sim_debug PRIVATE -DWIN32)
endif()

# Output to build directory (MSVC will add Debug/Release subdirs)
set_target_properties(sim_debug PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${SIM_ROOT}/build/Debug
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${SIM_ROOT}/build/Release
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${SIM_ROOT}/build/RelWithDebInfo
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL ${SIM_ROOT}/build/MinSizeRel
    OUTPUT_NAME "sim_debug"
)

# Link math library if needed
if (UNIX)
    target_link_libraries(sim_debug m)
endif()