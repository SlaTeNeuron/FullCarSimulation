// Vehicle Dynamics Engine - Debug Test Harness
//
// Drives the simulation through the Unity API (unity_api.h) directly, giving
// code-path parity with the DLL build.  Every VehicleSim_* call here is
// the exact function that ships in racing_sim.dll — no internal shortcuts.
//
// Build note (CMakeLists):
//   sim_debug compiles unity_api.c statically alongside this file.
//   BUILDING_DLL is defined so UNITY_API expands to dllexport (harmless on an exe).

#include "unity_api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Default asset paths (relative to the executable's working directory when
// launched from build/Debug/ or build/Release/).
#define DEFAULT_VEHICLE_PATH  "../../data/vehicles/TBReCar.txt"
#define DEFAULT_TRACK_PATH    "../../data/tracks/skidpad.txt"

//-------------------------
// Real-time Log Callback
//-------------------------

// Mirrors what a Unity MonoBehaviour wires up via VehicleSim_SetLogCallback.
// Written to stderr so it doesn't interleave with test output on stdout.
static void debug_log_callback(SimErrorLevel level, const char* message, void* user_data) {
    (void)user_data;
    const char* tag;
    switch (level) {
        case SIM_LOG_DEBUG: tag = "[DBG]"; break;
        case SIM_LOG_INFO:  tag = "[INF]"; break;
        case SIM_LOG_WARN:  tag = "[WRN]"; break;
        case SIM_LOG_ERROR: tag = "[ERR]"; break;
        default:            tag = "[???]"; break;
    }
    fprintf(stderr, "%s %s\n", tag, message);
}

//-------------------------
// Telemetry Callback
//-------------------------

// Matches TelemetryCallback: void (*)(const TelemetryFrame* frame, void* user_data).
// Called every step from inside VehicleSim_Step — same as in the DLL.
static int telemetry_count = 0;
static void debug_telemetry_callback(const TelemetryFrame* frame, void* user_data) {
    (void)user_data;
    telemetry_count++;
    if (telemetry_count % 10 == 0) {
        printf("  [Telem #%4d | T=%7.3f s]  pos=(%.2f,%.2f,%.2f)  "
               "spd=%5.2f m/s  thr=%.2f  brk=%.2f  str=%+.2f\n",
               telemetry_count, frame->time,
               frame->position[0], frame->position[1], frame->position[2],
               frame->speed,
               frame->throttle, frame->brake, frame->steering);
    }
}

//-------------------------
// State / Render Printing
//-------------------------

// Derive a scalar yaw angle (deg) from the Unity-space orientation quaternion
// [w, x, y, z] stored in TelemetryFrame.orientation[].
// Yaw = rotation about the Unity Y-up axis (left-handed sense matches Unity's
// Euler.y decomposition).
static double yaw_from_quat(const double* q /* [w,x,y,z] */) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    double yaw_rad = atan2(2.0 * (w * y + x * z),
                           1.0 - 2.0 * (y * y + z * z));
    return yaw_rad * 57.29577951;  // rad → deg
}

// Print a formatted state snapshot obtained via VehicleSim_GetTelemetry.
static void print_state(VehicleSimulation* sim, int step_idx) {
    TelemetryFrame frame;
    VehicleSim_GetTelemetry(sim, &frame);

    printf("\n--- Step %d | T=%7.3f s ---\n", step_idx, frame.time);
    // Unity space: X = right, Y = up, Z = forward
    printf("  Pos  [R=%7.3f  U=%7.3f  F=%7.3f] m\n",
           frame.position[0], frame.position[1], frame.position[2]);
    printf("  Vel  [R=%7.3f  U=%7.3f  F=%7.3f] m/s  |  "
           "Speed: %.3f m/s (%.1f km/h)\n",
           frame.velocity[0], frame.velocity[1], frame.velocity[2],
           frame.speed, frame.speed * 3.6);
    printf("  Yaw: %+7.2f deg  |  Yaw rate: %+.4f rad/s\n",
           yaw_from_quat(frame.orientation), frame.yaw_rate);
    printf("  Inputs: thr=%.2f  brk=%.2f  str=%+.2f  gear=%d\n",
           frame.throttle, frame.brake, frame.steering, frame.gear);
    printf("  Wheels ω FL/FR/RL/RR: %6.2f / %6.2f / %6.2f / %6.2f rad/s\n",
           frame.wheel_angular_vel[0], frame.wheel_angular_vel[1],
           frame.wheel_angular_vel[2], frame.wheel_angular_vel[3]);
    printf("  Susp  Δ FL/FR/RL/RR: %+.4f / %+.4f / %+.4f / %+.4f m\n",
           frame.suspension_deflection[0], frame.suspension_deflection[1],
           frame.suspension_deflection[2], frame.suspension_deflection[3]);
}

// Print a compact chassis render snapshot (position, orientation, speed).
// This exercises the same VehicleSim_GetRenderData path Unity calls every frame.
static void print_render_data(VehicleSimulation* sim) {
    VehicleRenderData rd;
    VehicleSim_GetRenderData(sim, &rd);
    printf("  [RenderData] pos=(%.3f, %.3f, %.3f)  "
           "q=(w=%.3f x=%.3f y=%.3f z=%.3f)  "
           "%.1f km/h\n",
           rd.chassis.position_x, rd.chassis.position_y, rd.chassis.position_z,
           rd.chassis.orientation_w, rd.chassis.orientation_x,
           rd.chassis.orientation_y, rd.chassis.orientation_z,
           rd.speed_kmh);
}

// Print VehicleParameters — confirms what the engine loaded.
static void print_vehicle_params(VehicleSimulation* sim) {
    VehicleParameters p;
    VehicleSim_GetVehicleParameters(sim, &p);
    printf("\n  Vehicle Parameters:\n");
    printf("    Mass: %.1f kg  |  Wheelbase: %.3f m  |  Track F/R: %.3f / %.3f m\n",
           p.mass, p.wheelbase, p.track_front, p.track_rear);
    printf("    CG height: %.3f m  |  CG→front: %.3f m  |  CG→rear: %.3f m\n",
           p.cg_height, p.cg_to_front, p.cg_to_rear);
    printf("    Spring F/R: %.0f / %.0f N/m  |  Damping F/R: %.0f / %.0f Ns/m\n",
           p.spring_rate_front, p.spring_rate_rear, p.damping_front, p.damping_rear);
    printf("    Tire radius: %.4f m  |  Friction μ: %.2f\n",
           p.tire_radius, p.tire_friction_coeff);
    printf("    Aero: Cd=%.2f  Cl=%.2f  Area=%.2f m²\n",
           p.drag_coefficient, p.lift_coefficient, p.frontal_area);
    printf("    Motor: max torque=%.1f Nm  |  max RPM=%.0f\n\n",
           p.max_engine_torque, p.max_rpm);
}

// Dump the full diagnostic blob to stderr after any initialization failure.
static void dump_diagnostics(VehicleSimulation* sim) {
    char diag[4096];
    VehicleSim_GetDiagnostics(sim, diag, (int)sizeof(diag));
    fprintf(stderr, "\n=== DIAGNOSTICS ===\n%s\n===================\n", diag);
}

//-------------------------
// Lifecycle Helper
//-------------------------

// Replicates the exact Unity startup sequence:
//   Create → SetLogCallback → LoadVehicle → LoadTrack → Initialize
// Returns a ready simulation handle, or NULL on any failure.
static VehicleSimulation* create_and_init(double timestep,
                                          const char* vehicle_path,
                                          const char* track_path) {
    VehicleSimulation* sim = VehicleSim_Create(timestep);
    if (!sim) {
        fprintf(stderr, "[debug] VehicleSim_Create returned NULL\n");
        return NULL;
    }

    // Register the log callback first — exactly as Unity does.
    VehicleSim_SetLogCallback(sim, debug_log_callback, NULL);

    if (VehicleSim_LoadVehicle(sim, vehicle_path) != 0) {
        fprintf(stderr, "[debug] LoadVehicle failed: %s\n",
                VehicleSim_GetLastError(sim));
        dump_diagnostics(sim);
        VehicleSim_Destroy(sim);
        return NULL;
    }

    if (VehicleSim_LoadTrack(sim, track_path) != 0) {
        fprintf(stderr, "[debug] LoadTrack failed: %s\n",
                VehicleSim_GetLastError(sim));
        dump_diagnostics(sim);
        VehicleSim_Destroy(sim);
        return NULL;
    }

    if (VehicleSim_Initialize(sim) != 0) {
        fprintf(stderr, "[debug] Initialize failed: %s\n",
                VehicleSim_GetLastError(sim));
        dump_diagnostics(sim);
        VehicleSim_Destroy(sim);
        return NULL;
    }

    return sim;
}

//-------------------------
// Test Scenarios
//-------------------------

// Each scenario operates on an already-initialized sim and returns without
// resetting it — time and position are cumulative across all four tests,
// which exercises the same stateful behaviour Unity sees.

static void test_acceleration(VehicleSimulation* sim, int steps) {
    printf("\n========================================\n");
    printf("TEST 1: Straight Line Acceleration\n");
    printf("========================================\n");

    DriverInputs inputs = {0};
    inputs.throttle = 1.0;
    inputs.gear     = 1;

    for (int i = 0; i < steps; i++) {
        VehicleSim_Step(sim, &inputs);
        if (i % 50 == 0 || i == steps - 1) {
            print_state(sim, i);
        }
    }
    print_render_data(sim);
}

static void test_braking(VehicleSimulation* sim, int steps) {
    printf("\n========================================\n");
    printf("TEST 2: Braking\n");
    printf("========================================\n");

    DriverInputs inputs = {0};
    inputs.gear = 1;

    // Accelerate for 100 steps first.
    inputs.throttle = 1.0;
    VehicleSim_StepMultiple(sim, &inputs, 100);
    printf("  [Accelerated 100 steps]\n");
    print_state(sim, 100);

    // Full brake.
    printf("  [Braking...]\n");
    inputs.throttle = 0.0;
    inputs.brake    = 1.0;
    for (int i = 0; i < steps; i++) {
        VehicleSim_Step(sim, &inputs);
        if (i % 25 == 0 || i == steps - 1) {
            print_state(sim, 100 + i);
        }
    }
    print_render_data(sim);
}

static void test_turning(VehicleSimulation* sim, int steps) {
    printf("\n========================================\n");
    printf("TEST 3: Turning Circle\n");
    printf("========================================\n");

    DriverInputs inputs = {0};
    inputs.throttle = 0.5;
    inputs.steering = 1.0;   // Full right steer
    inputs.gear     = 1;

    for (int i = 0; i < steps; i++) {
        VehicleSim_Step(sim, &inputs);
        if (i % 50 == 0 || i == steps - 1) {
            print_state(sim, i);
        }
    }
    print_render_data(sim);
}

static void test_slalom(VehicleSimulation* sim, int steps) {
    printf("\n========================================\n");
    printf("TEST 4: Slalom Pattern\n");
    printf("========================================\n");

    DriverInputs inputs = {0};
    inputs.throttle = 0.6;
    inputs.gear     = 1;

    for (int i = 0; i < steps; i++) {
        inputs.steering = ((i / 50) % 2 == 0) ? 0.7 : -0.7;
        VehicleSim_Step(sim, &inputs);
        if (i % 50 == 0 || i == steps - 1) {
            print_state(sim, i);
        }
    }
    print_render_data(sim);
}

//-------------------------
// Interactive Mode (REPL)
//-------------------------

static void run_interactive(VehicleSimulation* sim) {
    printf("\n========================================\n");
    printf("INTERACTIVE MODE\n");
    printf("========================================\n");
    printf("Commands:\n");
    printf("  step [N]          Run N physics steps (default 1)\n");
    printf("  input T B S [G]   Set throttle brake steer [gear]\n");
    printf("  reset             Reset + re-initialize simulation\n");
    printf("  state             Print telemetry state\n");
    printf("  render            Print render data\n");
    printf("  telem             Print full telemetry frame\n");
    printf("  params            Print vehicle parameters\n");
    printf("  validate          Run validation check\n");
    printf("  stats             Print performance statistics\n");
    printf("  diag              Dump full diagnostics to stderr\n");
    printf("  quit / exit       Exit\n\n");

    char         cmd[256];
    int          step_count = 0;
    DriverInputs inputs     = {0};
    inputs.gear = 1;

    while (1) {
        printf("> ");
        fflush(stdout);
        if (!fgets(cmd, sizeof(cmd), stdin)) break;

        if (strncmp(cmd, "step", 4) == 0) {
            int n = 1;
            sscanf(cmd + 4, "%d", &n);
            if (n <= 0) n = 1;
            VehicleSim_StepMultiple(sim, &inputs, n);
            step_count += n;
            printf("  Ran %d step(s) (total: %d)\n", n, step_count);
            print_state(sim, step_count);

        } else if (strncmp(cmd, "input", 5) == 0) {
            double t = 0.0, b = 0.0, s = 0.0;
            int    g = inputs.gear;
            if (sscanf(cmd + 5, "%lf %lf %lf %d", &t, &b, &s, &g) >= 3) {
                inputs.throttle = t;
                inputs.brake    = b;
                inputs.steering = s;
                inputs.gear     = g;
                printf("  Inputs: thr=%.2f  brk=%.2f  str=%+.2f  gear=%d\n",
                       t, b, s, g);
            } else {
                printf("  Usage: input <throttle> <brake> <steer> [gear]\n");
            }

        } else if (strncmp(cmd, "reset", 5) == 0) {
            VehicleSim_Reset(sim);
            if (VehicleSim_Initialize(sim) == 0) {
                step_count = 0;
                printf("  Reset + re-initialized OK.\n");
            } else {
                fprintf(stderr, "  Initialize after reset failed: %s\n",
                        VehicleSim_GetLastError(sim));
                dump_diagnostics(sim);
            }

        } else if (strncmp(cmd, "state", 5) == 0) {
            print_state(sim, step_count);

        } else if (strncmp(cmd, "render", 6) == 0) {
            print_render_data(sim);

        } else if (strncmp(cmd, "telem", 5) == 0) {
            TelemetryFrame frame;
            VehicleSim_GetTelemetry(sim, &frame);
            print_state(sim, step_count);
            printf("  Orient quat: (w=%.4f  x=%.4f  y=%.4f  z=%.4f)\n",
                   frame.orientation[0], frame.orientation[1],
                   frame.orientation[2], frame.orientation[3]);
            printf("  Accel [R/U/F]: (%.4f  %.4f  %.4f) m/s²\n",
                   frame.acceleration[0], frame.acceleration[1], frame.acceleration[2]);
            printf("  Ang vel [R/U/F]: (%.4f  %.4f  %.4f) rad/s\n",
                   frame.angular_velocity[0], frame.angular_velocity[1],
                   frame.angular_velocity[2]);

        } else if (strncmp(cmd, "params", 6) == 0) {
            print_vehicle_params(sim);

        } else if (strncmp(cmd, "validate", 8) == 0) {
            ValidationResult vr;
            VehicleSim_Validate(sim, &vr);
            printf("  Validation: %s (code=%d) — %s\n",
                   vr.is_valid ? "PASS" : "FAIL", vr.error_code, vr.message);

        } else if (strncmp(cmd, "stats", 5) == 0) {
            SimulationStats stats;
            VehicleSim_GetStats(sim, &stats);
            printf("  Steps=%d  avg=%.4f ms  max=%.4f ms  RTF=%.2fx\n",
                   stats.total_steps,
                   stats.average_step_time_ms,
                   stats.max_step_time_ms,
                   stats.real_time_factor);

        } else if (strncmp(cmd, "diag", 4) == 0) {
            dump_diagnostics(sim);

        } else if (strncmp(cmd, "quit", 4) == 0 ||
                   strncmp(cmd, "exit", 4) == 0) {
            break;

        } else if (cmd[0] != '\n' && cmd[0] != '\r' && cmd[0] != '\0') {
            printf("  Unknown command. Type 'quit' to exit.\n");
        }
    }
}

//-------------------------
// Main Entry Point
//-------------------------

int main(int argc, char** argv) {
    printf("========================================\n");
    printf("Racing Simulation - Debug Test Harness\n");
    printf("(Unity API code path — identical to DLL)\n");
    printf("========================================\n\n");

    double      timestep     = 0.001;               // 1 kHz — same as recommended DLL rate
    int         test_mode    = 0;                   // 0 = all, 1-4 = specific, -1 = interactive
    const char* vehicle_path = DEFAULT_VEHICLE_PATH;
    const char* track_path   = DEFAULT_TRACK_PATH;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-dt") == 0 && i + 1 < argc) {
            timestep = atof(argv[++i]);
        } else if (strcmp(argv[i], "-test") == 0 && i + 1 < argc) {
            test_mode = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-interactive") == 0 ||
                   strcmp(argv[i], "-i") == 0) {
            test_mode = -1;
        } else if (strcmp(argv[i], "-vehicle") == 0 && i + 1 < argc) {
            vehicle_path = argv[++i];
        } else if (strcmp(argv[i], "-track") == 0 && i + 1 < argc) {
            track_path = argv[++i];
        } else if (strcmp(argv[i], "-help") == 0 ||
                   strcmp(argv[i], "-h") == 0) {
            printf("Usage: %s [options]\n\n", argv[0]);
            printf("  -dt <sec>          Physics timestep (default: %.4f)\n", timestep);
            printf("  -test <1-4>        Run one test: 1=accel 2=brake 3=turn 4=slalom\n");
            printf("  -interactive, -i   Interactive REPL\n");
            printf("  -vehicle <path>    Vehicle config (default: %s)\n", DEFAULT_VEHICLE_PATH);
            printf("  -track   <path>    Track config   (default: %s)\n", DEFAULT_TRACK_PATH);
            printf("  -help, -h          This message\n");
            return 0;
        }
    }

    printf("Config: dt=%.4f s (%.0f Hz)  vehicle=%s  track=%s\n\n",
           timestep, 1.0 / timestep, vehicle_path, track_path);

    // ---- Startup (mirrors Unity Start()) ----
    VehicleSimulation* sim = create_and_init(timestep, vehicle_path, track_path);
    if (!sim) {
        fprintf(stderr, "ERROR: Initialization failed — exiting.\n");
        return 1;
    }

    print_vehicle_params(sim);

    // Register telemetry callback (mirrors VehicleSim_RegisterTelemetryCallback in Unity).
    VehicleSim_RegisterTelemetryCallback(sim, debug_telemetry_callback, NULL);

    // Initial validation before stepping.
    {
        ValidationResult vr;
        VehicleSim_Validate(sim, &vr);
        printf("Initial validation: %s — %s\n",
               vr.is_valid ? "PASS" : "FAIL", vr.message);
        if (!vr.is_valid) {
            dump_diagnostics(sim);
            VehicleSim_Destroy(sim);
            return 1;
        }
    }

    // ---- Run test scenarios ----
    if (test_mode == -1) {
        run_interactive(sim);
    } else if (test_mode == 0) {
        test_acceleration(sim, 200);
        test_braking(sim, 100);
        test_turning(sim, 300);
        test_slalom(sim, 300);
    } else {
        switch (test_mode) {
            case 1: test_acceleration(sim, 200); break;
            case 2: test_braking(sim,   100); break;
            case 3: test_turning(sim,   300); break;
            case 4: test_slalom(sim,    300); break;
            default: printf("Invalid test %d — use 1-4\n", test_mode); break;
        }
    }

    // ---- Post-run summary ----
    printf("\n========================================\n");
    printf("Run complete.\n");
    {
        SimulationStats stats;
        VehicleSim_GetStats(sim, &stats);
        printf("  Total steps      : %d\n",    stats.total_steps);
        printf("  Avg step time    : %.4f ms\n", stats.average_step_time_ms);
        printf("  Max step time    : %.4f ms\n", stats.max_step_time_ms);
        printf("  Real-time factor : %.2fx\n",  stats.real_time_factor);
        printf("  Telemetry frames : %d\n",     telemetry_count);
    }
    {
        ValidationResult vr;
        VehicleSim_Validate(sim, &vr);
        printf("  Final validation : %s — %s\n",
               vr.is_valid ? "PASS" : "FAIL", vr.message);
    }
    printf("  Sim time         : %.4f s\n", VehicleSim_GetTime(sim));

    // ---- Cleanup (mirrors Unity OnDestroy()) ----
    VehicleSim_Destroy(sim);
    printf("\nDebug run complete!\n");

#ifdef _WIN32
    printf("\nPress Enter to exit...");
    getchar();
#endif

    return 0;
}
