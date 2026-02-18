#include "simulation/simulation_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//-------------------------
// Debug Helpers
//-------------------------

/* Debug telemetry callback */
static int telemetry_count = 0;
static void debug_telemetry_callback(const vde_real* data, int len, void* user) {
    telemetry_count++;
    if (telemetry_count % 10 == 0) {  /* Print every 10th frame */
        printf("  [Telemetry #%d] ", telemetry_count);
        for (int i = 0; i < len; i++) {
            printf("%.3f ", (double)data[i]);
        }
        printf("\n");
    }
}

/* Print simulation state */
static void print_state(Sim* sim, int step) {
    SimState state;
    sim_get_state(sim, &state);
    
    printf("\n=== Step %d | Time: %.3f s ===\n", step, (double)state.time);
    printf("Position: (%.3f, %.3f, %.3f) m\n", (double)state.px, (double)state.py, (double)state.pz);
    printf("Orientation: yaw=%.2f° pitch=%.2f° roll=%.2f°\n", 
           (double)state.yaw * 57.2957795, (double)state.pitch * 57.2957795, (double)state.roll * 57.2957795);
    printf("Velocity: %.3f m/s | Accel: %.3f m/s²\n", (double)state.vel, (double)state.accel);
    printf("Steering: %.2f° | RPM: %.1f\n", (double)state.steeringAngle * 57.2957795, (double)state.motorRPM);
}

//-------------------------
// Test Scenarios
//-------------------------

/* Test scenario: straight line acceleration */
static void test_acceleration(Sim* sim, int steps) {
    printf("\n========================================\n");
    printf("TEST 1: Straight Line Acceleration\n");
    printf("========================================\n");
    
    sim_set_inputs(sim, 1.0, 0.0, 0.0);  /* Full throttle, no brake, no steer */
    sim_start(sim);
    
    for (int i = 0; i < steps; i++) {
        sim_step(sim);
        if (i % 50 == 0 || i == steps - 1) {
            print_state(sim, i);
        }
    }
    
    sim_stop(sim);
}

/* Test scenario: braking */
static void test_braking(Sim* sim, int steps) {
    printf("\n========================================\n");
    printf("TEST 2: Braking\n");
    printf("========================================\n");
    
    /* First accelerate */
    sim_set_inputs(sim, 1.0, 0.0, 0.0);
    sim_start(sim);
    
    for (int i = 0; i < 100; i++) {
        sim_step(sim);
    }
    
    printf("Reached speed, now braking...\n");
    
    /* Now brake */
    sim_set_inputs(sim, 0.0, 1.0, 0.0);  /* No throttle, full brake */
    
    for (int i = 0; i < steps; i++) {
        sim_step(sim);
        if (i % 25 == 0 || i == steps - 1) {
            print_state(sim, 100 + i);
        }
    }
    
    sim_stop(sim);
}

/* Test scenario: circle/turning */
static void test_turning(Sim* sim, int steps) {
    printf("\n========================================\n");
    printf("TEST 3: Turning Circle\n");
    printf("========================================\n");
    
    sim_set_inputs(sim, 0.5, 0.0, 1.0);  /* Half throttle, full right steer */
    sim_start(sim);
    
    for (int i = 0; i < steps; i++) {
        sim_step(sim);
        if (i % 50 == 0 || i == steps - 1) {
            print_state(sim, i);
        }
    }
    
    sim_stop(sim);
}

/* Test scenario: slalom/weaving */
static void test_slalom(Sim* sim, int steps) {
    printf("\n========================================\n");
    printf("TEST 4: Slalom Pattern\n");
    printf("========================================\n");
    
    sim_start(sim);
    
    for (int i = 0; i < steps; i++) {
        /* Alternate steering every 50 steps */
        vde_real steer = ((i / 50) % 2 == 0) ? (vde_real)0.7 : (vde_real)-0.7;
        sim_set_inputs(sim, (vde_real)0.6, (vde_real)0.0, steer);
        
        sim_step(sim);
        
        if (i % 50 == 0 || i == steps - 1) {
            print_state(sim, i);
        }
    }
    
    sim_stop(sim);
}

/* Interactive mode */
static void run_interactive(Sim* sim) {
    printf("\n========================================\n");
    printf("INTERACTIVE MODE\n");
    printf("========================================\n");
    printf("Commands:\n");
    printf("  step N       - Run N simulation steps\n");
    printf("  input T B S  - Set throttle, brake, steer (each -1 to 1)\n");
    printf("  start        - Start simulation\n");
    printf("  stop         - Stop simulation\n");
    printf("  state        - Print current state\n");
    printf("  telem        - Get telemetry\n");
    printf("  quit         - Exit\n\n");
    
    char cmd[256];
    int step_count = 0;
    
    while (1) {
        printf("> ");
        if (!fgets(cmd, sizeof(cmd), stdin)) break;
        
        if (strncmp(cmd, "step", 4) == 0) {
            int n = 1;
            sscanf(cmd + 4, "%d", &n);
            for (int i = 0; i < n; i++) {
                sim_step(sim);
                step_count++;
            }
            printf("Executed %d steps (total: %d)\n", n, step_count);
            print_state(sim, step_count);
            
        } else if (strncmp(cmd, "input", 5) == 0) {
            double throttle_d, brake_d, steer_d;
            if (sscanf(cmd + 5, "%lf %lf %lf", &throttle_d, &brake_d, &steer_d) == 3) {
                vde_real throttle = (vde_real)throttle_d;
                vde_real brake = (vde_real)brake_d;
                vde_real steer = (vde_real)steer_d;
                sim_set_inputs(sim, throttle, brake, steer);
                printf("Set inputs: throttle=%.2f, brake=%.2f, steer=%.2f\n", 
                       throttle_d, brake_d, steer_d);
            } else {
                printf("Usage: input <throttle> <brake> <steer>\n");
            }
            
        } else if (strncmp(cmd, "start", 5) == 0) {
            sim_start(sim);
            printf("Simulation started\n");
            
        } else if (strncmp(cmd, "stop", 4) == 0) {
            sim_stop(sim);
            printf("Simulation stopped\n");
            
        } else if (strncmp(cmd, "state", 5) == 0) {
            print_state(sim, step_count);
            
        } else if (strncmp(cmd, "telem", 5) == 0) {
            vde_real buffer[16];
            int len = sim_get_latest_telemetry(sim, buffer, 16);
            printf("Telemetry (%d values): ", len);
            for (int i = 0; i < len; i++) {
                printf("%.3f ", (double)buffer[i]);
            }
            printf("\n");
            
        } else if (strncmp(cmd, "quit", 4) == 0) {
            break;
            
        } else {
            printf("Unknown command\n");
        }
    }
}

//-------------------------
// Main Entry Point
//-------------------------

/* Main debug entry point */
int main(int argc, char** argv) {
    printf("========================================\n");
    printf("Racing Simulation - Debug Test Environment\n");
    printf("========================================\n\n");
    
    /* Parse command line */
    vde_real timestep = (vde_real)0.02;  /* 50 Hz default */
    int test_mode = 0;                    /* 0 = all tests, 1-4 = specific test, -1 = interactive */
    
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-dt") == 0 && i + 1 < argc) {
            timestep = (vde_real)atof(argv[i + 1]);
            i++;
        } else if (strcmp(argv[i], "-test") == 0 && i + 1 < argc) {
            test_mode = atoi(argv[i + 1]);
            i++;
        } else if (strcmp(argv[i], "-interactive") == 0 || strcmp(argv[i], "-i") == 0) {
            test_mode = -1;
        } else if (strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "-h") == 0) {
            printf("Usage: %s [options]\n", argv[0]);
            printf("Options:\n");
            printf("  -dt <seconds>    Set timestep (default: 0.02)\n");
            printf("  -test <1-4>      Run specific test (1=accel, 2=brake, 3=turn, 4=slalom)\n");
            printf("  -interactive     Run in interactive mode\n");
            printf("  -help            Show this help\n");
            return 0;
        }
    }
    
    printf("Creating simulation with dt=%.4f s (%.1f Hz)\n", (double)timestep, 1.0/(double)timestep);
    
    /* Create simulation */
    Sim* sim = sim_create(timestep);
    if (!sim) {
        fprintf(stderr, "ERROR: Failed to create simulation\n");
        return 1;
    }
    
    /* Register telemetry callback */
    sim_register_telemetry_cb(sim, debug_telemetry_callback, NULL);
    
    /* Run tests */
    if (test_mode == -1) {
        /* Interactive mode */
        run_interactive(sim);
    } else if (test_mode == 0) {
        /* Run all tests */
        test_acceleration(sim, 200);
        test_braking(sim, 100);
        test_turning(sim, 300);
        test_slalom(sim, 300);
    } else {
        /* Run specific test */
        switch (test_mode) {
            case 1: test_acceleration(sim, 200); break;
            case 2: test_braking(sim, 100); break;
            case 3: test_turning(sim, 300); break;
            case 4: test_slalom(sim, 300); break;
            default:
                printf("Invalid test number. Use 1-4\n");
        }
    }
    
    /* Cleanup */
    printf("\n========================================\n");
    printf("Cleaning up and exiting...\n");
    printf("Total telemetry callbacks: %d\n", telemetry_count);
    sim_destroy(sim);
    
    printf("Debug test complete!\n");
    
    /* Pause so user can see output when double-clicking exe */
    #ifdef _WIN32
    printf("\nPress any key to exit...");
    getchar();
    #endif
    
    return 0;
}
