#include "vehicle/vehicle_parameters.h"
#include "vehicle/suspension.h"
#include "vehicle/wheel.h"
#include "vehicle/brakes.h"
#include "vehicle/steering.h"
#include "vehicle/driveline.h"
#include "vehicle/aerodynamics.h"
#include "vehicle/sprung_mass.h"
#include "tire_models/magic_formula.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

//-------------------------
// Compile-Time TBRe Defaults
//
// This static const lives in read-only memory.
// vehicle_params_get_defaults() returns a pointer to it — zero cost.
// All values are for the TBRe FSAE competition car.
//
// Guiggiani sign/axis convention:
//   X = forward, Y = left, Z = up (vehicle body frame)
//   cg_to_front_axle = a  (forward),  cg_to_rear_axle = c  (rearward)
//   Static load fraction: N_f/W = c/L  (rear axle distance determines front load)
//-------------------------

static const VehicleParameters s_tbre_defaults = {

    .name    = "TBRe FSAE Car",
    .version = "1.0",

    //-------- Mass & Inertia --------
    // 230 kg chassis + 50 kg driver = 280 kg total
    // 45% front / 55% rear weight distribution
    .mass = {
        .total_mass         = 280.0,
        .sprung_mass        = 230.0,
        .unsprung_mass      = { 12.5, 12.5, 12.5, 12.5 },  // [FL,FR,RL,RR]
        .inertia_roll       =  25.0,   // Ixx  kg·m²  (narrow track, low CG)
        .inertia_pitch      =  75.0,   // Iyy  kg·m²
        .inertia_yaw        =  90.0,   // Izz  kg·m²
        .cg_height          =   0.265  // m    above ground
    },

    //-------- Chassis Geometry --------
    // Wheelbase 1.530 m, 45/55 weight split:
    //   N_f/W = c/L = 0.45  →  c (CG→rear)  = 0.689 m
    //                          a (CG→front) = 0.841 m
    .geometry = {
        .wheelbase          = 1.530,
        .track_front        = 1.220,
        .track_rear         = 1.170,
        .cg_to_front_axle   = 0.841,   // a (m)
        .cg_to_rear_axle    = 0.689    // c (m)
    },

    //-------- Suspension --------
    // Stiff FSAE setup; springs stiffer at front for handling balance.
    // Corners ordered [FL, FR, RL, RR].
    .suspension = {
        .spring_rate        = { 35000.0, 35000.0, 30000.0, 30000.0 },  // N/m
        .damper_coeff       = {  2000.0,  2000.0,  1800.0,  1800.0 },  // N·s/m
        .rest_length        = {   0.200,   0.200,   0.200,   0.200 },  // m
        .roll_stiffness_front = 800.0,   // N·m/rad  (stiffer front ARB for understeer)
        .roll_stiffness_rear  = 400.0    // N·m/rad
    },

    //-------- Wheel & Tyre --------
    // Hoosier 20.5×7.0-13 FSAE slick:
    //   OD = 20.5 in = 0.521 m  →  radius = 0.260 m
    //   Width = 7.0 in = 0.178 m
    .wheel = {
        .radius             = 0.260,
        .width              = 0.178,
        .inertia            = 0.9,     // kg·m²  per wheel (small 13" rim)
        .rolling_resistance = 0.012    // Crr (racing slick on smooth tarmac)
    },

    //-------- Aerodynamics --------
    // Full aero package (front wing + diffuser + rear wing)
    // At 30 m/s (~108 km/h):
    //   Downforce = 0.5 × 1.225 × 0.92 × 2.10 × 900 ≈ 1060 N (≈ 38 % of weight)
    .aero = {
        .drag_coeff         =  1.35,
        .lift_coeff         = -2.10,   // negative = downforce
        .frontal_area       =  0.92,   // m²
        .aero_balance       =  0.40,   // 40 % front / 60 % rear downforce
        .air_density        =  1.225   // kg/m³  (sea level, 15 °C)
    },

    //-------- Brakes --------
    // Hydraulic, large-piston calipers; front-biased for load-transfer benefit.
    // Guiggiani Sec. 4.6: optimal balance φ = c/L + μ·h/L
    .brakes = {
        .max_torque_front   = 1200.0,  // N·m per front wheel
        .max_torque_rear    =  800.0,  // N·m per rear  wheel
        .brake_balance      =    0.65  // 65 % front
    },

    //-------- Steering --------
    // Short direct rack; Ackermann factor 0.75 (partial geometry).
    .steering = {
        .ratio              =  3.5,
        .max_angle          =  0.349,  // rad ≈ 20°
        .ackermann_factor   =  0.75
    },

    //-------- Driveline --------
    // Yamaha R6 600 cc engine with 19 mm intake restrictor.
    // Rear-wheel drive, 5-speed sequential, chain final drive.
    // gear_ratios[0..5] = 1st..6th  (6th unused → 0.0), [6] = reverse
    .driveline = {
        .drive_config          = DRIVE_RWD,
        .engine_max_torque     =  72.0,  // N·m (restricted, peak at ~11 000 rpm)
        .gear_ratios           = { 2.615, 1.812, 1.409, 1.130, 0.935, 0.0, -3.0 },
        .final_drive_ratio     =  3.77,  // chain sprocket ratio
        .drivetrain_efficiency =  0.91   // chain drivetrain (~9 % losses)
    },

    //-------- Tyre Model (Pacejka MF, Hoosier FSAE slick) --------
    // Tuned to give:
    //   Long. peak force: μx ≈ 1.80 at peak slip σ ≈ 10 %
    //   Lat.  peak force: μy ≈ 1.90 at peak angle α ≈ 7°
    //   Cornering stiffness ≈ 14 000 N/rad at Fz = 700 N
    //
    // Longitudinal (b-series)
    //   Cx = b0 = 1.65 (shape)
    //   Dx = Fz × (b1×Fz + b2)  →  b2 ≈ μx_peak
    //   Kx = Fz × (b3×Fz + b4) × exp(−b5×Fz)
    //   Ex = b6×Fz² + b7×Fz + b8
    // Lateral (a-series)
    //   Cy = a0 = 1.30 (shape)
    //   Dy = Fz × (a1×Fz + a2)  →  a2 ≈ μy_peak
    //   Ky = Fz × a3 × sin(a4 × atan(Fz / a5))
    //   Ey = a6×Fz + a7
    .tire_model = {
        .b0  =  1.65,
        .b1  =  0.0,
        .b2  =  1.80,
        .b3  =  0.0,
        .b4  = 28.0,   // → peak slip ≈ 10 %
        .b5  =  0.0,
        .b6  =  0.0,
        .b7  =  0.0,
        .b8  = -0.50,  // curvature (post-peak shape)
        .b9  =  0.0,
        .b10 =  0.0,

        .a0  =  1.30,
        .a1  =  0.0,
        .a2  =  1.90,
        .a3  = 22.0,   // → cornering stiffness ≈ 14 000 N/rad at 700 N
        .a4  =  1.50,
        .a5  = 700.0,  // reference load matches typical FSAE corner load (N)
        .a6  =  0.0,
        .a7  = -0.60,  // curvature
        .a8  =  0.0,

        .vertical_stiffness = 150000.0  // N/m (stiff slick carcass)
    }
};

//-------------------------
// Internal Helpers
//-------------------------

// Trim leading and trailing whitespace in-place.  Returns pointer to trimmed start.
static char* trim(char* s) {
    while (*s && isspace((unsigned char)*s)) s++;
    char* end = s + strlen(s);
    while (end > s && isspace((unsigned char)*(end - 1))) { end--; }
    *end = '\0';
    return s;
}

//-------------------------
// Lifecycle
//-------------------------

/**
 * Return a pointer to the compile-time TBRe default parameters.
 * Zero allocation — safe to call at any time, from any thread.
 */
const VehicleParameters* vehicle_params_get_defaults(void) {
    return &s_tbre_defaults;
}

/**
 * Allocate and initialise a VehicleParameters from the TBRe defaults.
 * Returns NULL on allocation failure.
 */
VehicleParameters* vehicle_params_create(void) {
    VehicleParameters* p = (VehicleParameters*)malloc(sizeof(VehicleParameters));
    if (!p) return NULL;
    memcpy(p, &s_tbre_defaults, sizeof(VehicleParameters));
    return p;
}

/**
 * Free a VehicleParameters created by vehicle_params_create().
 */
void vehicle_params_destroy(VehicleParameters* params) {
    if (!params) return;
    free(params);
}

//-------------------------
// File I/O — INI-Style Parser
//
// Format:
//   # comment
//   [section]
//   key = value
//
// Sections: [meta], [mass], [geometry], [suspension], [wheel],
//           [aero], [brakes], [steering], [driveline], [tire_model]
//
// Any key not matching a known parameter is counted as a warning and
// skipped (non-fatal).
//-------------------------

int vehicle_params_load_from_file(VehicleParameters* params, const char* filename) {
    if (!params || !filename) return -1;

    FILE* f = fopen(filename, "r");
    if (!f) return -1;

    char line[512];
    char section[64] = "";
    int  warnings    = 0;

    while (fgets(line, sizeof(line), f)) {
        // Strip newline / CR
        line[strcspn(line, "\r\n")] = '\0';
        char* s = trim(line);

        // Skip empty lines and comments
        if (*s == '\0' || *s == '#' || *s == ';') continue;

        // Section header: [section_name]
        if (*s == '[') {
            char* end = strchr(s, ']');
            if (end) {
                *end = '\0';
                strncpy(section, trim(s + 1), sizeof(section) - 1);
                section[sizeof(section) - 1] = '\0';
            }
            continue;
        }

        // Key = value pair
        char* eq = strchr(s, '=');
        if (!eq) continue;
        *eq = '\0';
        char* key = trim(s);
        char* val = trim(eq + 1);

        // Strip inline comment from value
        char* comment = strchr(val, '#');
        if (!comment) comment = strchr(val, ';');
        if (comment) { *comment = '\0'; val = trim(val); }

        double v = 0.0;
        int    vi = 0;
        int    parsed_float = (sscanf(val, "%lf", &v) == 1);
        int    parsed_int   = (sscanf(val, "%d",  &vi) == 1);

        int recognised = 1;  // assume recognised unless we fall through

        // ---- [meta] ----
        if (strcmp(section, "meta") == 0) {
            if      (strcmp(key, "name")    == 0) { strncpy(params->name,    val, sizeof(params->name)    - 1); }
            else if (strcmp(key, "version") == 0) { strncpy(params->version, val, sizeof(params->version) - 1); }
            else recognised = 0;
        }

        // ---- [mass] ----
        else if (strcmp(section, "mass") == 0) {
            if      (strcmp(key, "total_mass")         == 0 && parsed_float) params->mass.total_mass        = (vde_real)v;
            else if (strcmp(key, "sprung_mass")        == 0 && parsed_float) params->mass.sprung_mass       = (vde_real)v;
            else if (strcmp(key, "unsprung_mass_fl")   == 0 && parsed_float) params->mass.unsprung_mass[0]  = (vde_real)v;
            else if (strcmp(key, "unsprung_mass_fr")   == 0 && parsed_float) params->mass.unsprung_mass[1]  = (vde_real)v;
            else if (strcmp(key, "unsprung_mass_rl")   == 0 && parsed_float) params->mass.unsprung_mass[2]  = (vde_real)v;
            else if (strcmp(key, "unsprung_mass_rr")   == 0 && parsed_float) params->mass.unsprung_mass[3]  = (vde_real)v;
            else if (strcmp(key, "inertia_roll")       == 0 && parsed_float) params->mass.inertia_roll      = (vde_real)v;
            else if (strcmp(key, "inertia_pitch")      == 0 && parsed_float) params->mass.inertia_pitch     = (vde_real)v;
            else if (strcmp(key, "inertia_yaw")        == 0 && parsed_float) params->mass.inertia_yaw       = (vde_real)v;
            else if (strcmp(key, "cg_height")          == 0 && parsed_float) params->mass.cg_height         = (vde_real)v;
            else recognised = 0;
        }

        // ---- [geometry] ----
        else if (strcmp(section, "geometry") == 0) {
            if      (strcmp(key, "wheelbase")         == 0 && parsed_float) params->geometry.wheelbase        = (vde_real)v;
            else if (strcmp(key, "track_front")       == 0 && parsed_float) params->geometry.track_front      = (vde_real)v;
            else if (strcmp(key, "track_rear")        == 0 && parsed_float) params->geometry.track_rear       = (vde_real)v;
            else if (strcmp(key, "cg_to_front_axle")  == 0 && parsed_float) params->geometry.cg_to_front_axle = (vde_real)v;
            else if (strcmp(key, "cg_to_rear_axle")   == 0 && parsed_float) params->geometry.cg_to_rear_axle  = (vde_real)v;
            else recognised = 0;
        }

        // ---- [suspension] ----
        else if (strcmp(section, "suspension") == 0) {
            if      (strcmp(key, "spring_rate_fl")       == 0 && parsed_float) params->suspension.spring_rate[0]       = (vde_real)v;
            else if (strcmp(key, "spring_rate_fr")       == 0 && parsed_float) params->suspension.spring_rate[1]       = (vde_real)v;
            else if (strcmp(key, "spring_rate_rl")       == 0 && parsed_float) params->suspension.spring_rate[2]       = (vde_real)v;
            else if (strcmp(key, "spring_rate_rr")       == 0 && parsed_float) params->suspension.spring_rate[3]       = (vde_real)v;
            else if (strcmp(key, "damper_coeff_fl")      == 0 && parsed_float) params->suspension.damper_coeff[0]      = (vde_real)v;
            else if (strcmp(key, "damper_coeff_fr")      == 0 && parsed_float) params->suspension.damper_coeff[1]      = (vde_real)v;
            else if (strcmp(key, "damper_coeff_rl")      == 0 && parsed_float) params->suspension.damper_coeff[2]      = (vde_real)v;
            else if (strcmp(key, "damper_coeff_rr")      == 0 && parsed_float) params->suspension.damper_coeff[3]      = (vde_real)v;
            else if (strcmp(key, "rest_length_fl")       == 0 && parsed_float) params->suspension.rest_length[0]       = (vde_real)v;
            else if (strcmp(key, "rest_length_fr")       == 0 && parsed_float) params->suspension.rest_length[1]       = (vde_real)v;
            else if (strcmp(key, "rest_length_rl")       == 0 && parsed_float) params->suspension.rest_length[2]       = (vde_real)v;
            else if (strcmp(key, "rest_length_rr")       == 0 && parsed_float) params->suspension.rest_length[3]       = (vde_real)v;
            else if (strcmp(key, "roll_stiffness_front") == 0 && parsed_float) params->suspension.roll_stiffness_front = (vde_real)v;
            else if (strcmp(key, "roll_stiffness_rear")  == 0 && parsed_float) params->suspension.roll_stiffness_rear  = (vde_real)v;
            else recognised = 0;
        }

        // ---- [wheel] ----
        else if (strcmp(section, "wheel") == 0) {
            if      (strcmp(key, "radius")             == 0 && parsed_float) params->wheel.radius             = (vde_real)v;
            else if (strcmp(key, "width")              == 0 && parsed_float) params->wheel.width              = (vde_real)v;
            else if (strcmp(key, "inertia")            == 0 && parsed_float) params->wheel.inertia            = (vde_real)v;
            else if (strcmp(key, "rolling_resistance") == 0 && parsed_float) params->wheel.rolling_resistance = (vde_real)v;
            else recognised = 0;
        }

        // ---- [aero] ----
        else if (strcmp(section, "aero") == 0) {
            if      (strcmp(key, "drag_coeff")   == 0 && parsed_float) params->aero.drag_coeff   = (vde_real)v;
            else if (strcmp(key, "lift_coeff")   == 0 && parsed_float) params->aero.lift_coeff   = (vde_real)v;
            else if (strcmp(key, "frontal_area") == 0 && parsed_float) params->aero.frontal_area = (vde_real)v;
            else if (strcmp(key, "aero_balance") == 0 && parsed_float) params->aero.aero_balance = (vde_real)v;
            else if (strcmp(key, "air_density")  == 0 && parsed_float) params->aero.air_density  = (vde_real)v;
            else recognised = 0;
        }

        // ---- [brakes] ----
        else if (strcmp(section, "brakes") == 0) {
            if      (strcmp(key, "max_torque_front") == 0 && parsed_float) params->brakes.max_torque_front = (vde_real)v;
            else if (strcmp(key, "max_torque_rear")  == 0 && parsed_float) params->brakes.max_torque_rear  = (vde_real)v;
            else if (strcmp(key, "brake_balance")    == 0 && parsed_float) params->brakes.brake_balance    = (vde_real)v;
            else recognised = 0;
        }

        // ---- [steering] ----
        else if (strcmp(section, "steering") == 0) {
            if      (strcmp(key, "ratio")            == 0 && parsed_float) params->steering.ratio            = (vde_real)v;
            else if (strcmp(key, "max_angle")        == 0 && parsed_float) params->steering.max_angle        = (vde_real)v;
            else if (strcmp(key, "ackermann_factor") == 0 && parsed_float) params->steering.ackermann_factor = (vde_real)v;
            else recognised = 0;
        }

        // ---- [driveline] ----
        else if (strcmp(section, "driveline") == 0) {
            if      (strcmp(key, "drive_config")           == 0 && parsed_int)   params->driveline.drive_config          = (DriveConfig)vi;
            else if (strcmp(key, "engine_max_torque")      == 0 && parsed_float) params->driveline.engine_max_torque     = (vde_real)v;
            else if (strcmp(key, "final_drive_ratio")      == 0 && parsed_float) params->driveline.final_drive_ratio     = (vde_real)v;
            else if (strcmp(key, "drivetrain_efficiency")  == 0 && parsed_float) params->driveline.drivetrain_efficiency = (vde_real)v;
            else if (strcmp(key, "gear_ratio_1")  == 0 && parsed_float) params->driveline.gear_ratios[0] = (vde_real)v;
            else if (strcmp(key, "gear_ratio_2")  == 0 && parsed_float) params->driveline.gear_ratios[1] = (vde_real)v;
            else if (strcmp(key, "gear_ratio_3")  == 0 && parsed_float) params->driveline.gear_ratios[2] = (vde_real)v;
            else if (strcmp(key, "gear_ratio_4")  == 0 && parsed_float) params->driveline.gear_ratios[3] = (vde_real)v;
            else if (strcmp(key, "gear_ratio_5")  == 0 && parsed_float) params->driveline.gear_ratios[4] = (vde_real)v;
            else if (strcmp(key, "gear_ratio_6")  == 0 && parsed_float) params->driveline.gear_ratios[5] = (vde_real)v;
            else if (strcmp(key, "gear_ratio_r")  == 0 && parsed_float) params->driveline.gear_ratios[6] = (vde_real)v;
            else recognised = 0;
        }

        // ---- [tire_model] ----
        else if (strcmp(section, "tire_model") == 0) {
            if      (strcmp(key, "b0")  == 0 && parsed_float) params->tire_model.b0  = (vde_real)v;
            else if (strcmp(key, "b1")  == 0 && parsed_float) params->tire_model.b1  = (vde_real)v;
            else if (strcmp(key, "b2")  == 0 && parsed_float) params->tire_model.b2  = (vde_real)v;
            else if (strcmp(key, "b3")  == 0 && parsed_float) params->tire_model.b3  = (vde_real)v;
            else if (strcmp(key, "b4")  == 0 && parsed_float) params->tire_model.b4  = (vde_real)v;
            else if (strcmp(key, "b5")  == 0 && parsed_float) params->tire_model.b5  = (vde_real)v;
            else if (strcmp(key, "b6")  == 0 && parsed_float) params->tire_model.b6  = (vde_real)v;
            else if (strcmp(key, "b7")  == 0 && parsed_float) params->tire_model.b7  = (vde_real)v;
            else if (strcmp(key, "b8")  == 0 && parsed_float) params->tire_model.b8  = (vde_real)v;
            else if (strcmp(key, "b9")  == 0 && parsed_float) params->tire_model.b9  = (vde_real)v;
            else if (strcmp(key, "b10") == 0 && parsed_float) params->tire_model.b10 = (vde_real)v;
            else if (strcmp(key, "a0")  == 0 && parsed_float) params->tire_model.a0  = (vde_real)v;
            else if (strcmp(key, "a1")  == 0 && parsed_float) params->tire_model.a1  = (vde_real)v;
            else if (strcmp(key, "a2")  == 0 && parsed_float) params->tire_model.a2  = (vde_real)v;
            else if (strcmp(key, "a3")  == 0 && parsed_float) params->tire_model.a3  = (vde_real)v;
            else if (strcmp(key, "a4")  == 0 && parsed_float) params->tire_model.a4  = (vde_real)v;
            else if (strcmp(key, "a5")  == 0 && parsed_float) params->tire_model.a5  = (vde_real)v;
            else if (strcmp(key, "a6")  == 0 && parsed_float) params->tire_model.a6  = (vde_real)v;
            else if (strcmp(key, "a7")  == 0 && parsed_float) params->tire_model.a7  = (vde_real)v;
            else if (strcmp(key, "a8")  == 0 && parsed_float) params->tire_model.a8  = (vde_real)v;
            else if (strcmp(key, "vertical_stiffness") == 0 && parsed_float)
                params->tire_model.vertical_stiffness = (vde_real)v;
            else recognised = 0;
        }

        else {
            // Unknown section — count as warning
            recognised = 0;
        }

        if (!recognised) warnings++;
    }

    fclose(f);
    return warnings;
}

/**
 * Write all parameters to an INI-style config file.
 * Returns 0 on success, -1 on failure.
 */
int vehicle_params_save_to_file(const VehicleParameters* params, const char* filename) {
    if (!params || !filename) return -1;

    FILE* f = fopen(filename, "w");
    if (!f) return -1;

    fprintf(f, "# Vehicle Dynamics Engine — Vehicle Parameters\n");
    fprintf(f, "# Generated by vehicle_params_save_to_file()\n\n");

    fprintf(f, "[meta]\n");
    fprintf(f, "name    = %s\n", params->name);
    fprintf(f, "version = %s\n\n", params->version);

    fprintf(f, "[mass]\n");
    fprintf(f, "total_mass         = %.4f\n", (double)params->mass.total_mass);
    fprintf(f, "sprung_mass        = %.4f\n", (double)params->mass.sprung_mass);
    fprintf(f, "unsprung_mass_fl   = %.4f\n", (double)params->mass.unsprung_mass[0]);
    fprintf(f, "unsprung_mass_fr   = %.4f\n", (double)params->mass.unsprung_mass[1]);
    fprintf(f, "unsprung_mass_rl   = %.4f\n", (double)params->mass.unsprung_mass[2]);
    fprintf(f, "unsprung_mass_rr   = %.4f\n", (double)params->mass.unsprung_mass[3]);
    fprintf(f, "inertia_roll       = %.4f\n", (double)params->mass.inertia_roll);
    fprintf(f, "inertia_pitch      = %.4f\n", (double)params->mass.inertia_pitch);
    fprintf(f, "inertia_yaw        = %.4f\n", (double)params->mass.inertia_yaw);
    fprintf(f, "cg_height          = %.4f\n\n", (double)params->mass.cg_height);

    fprintf(f, "[geometry]\n");
    fprintf(f, "wheelbase          = %.4f\n", (double)params->geometry.wheelbase);
    fprintf(f, "track_front        = %.4f\n", (double)params->geometry.track_front);
    fprintf(f, "track_rear         = %.4f\n", (double)params->geometry.track_rear);
    fprintf(f, "cg_to_front_axle   = %.4f\n", (double)params->geometry.cg_to_front_axle);
    fprintf(f, "cg_to_rear_axle    = %.4f\n\n", (double)params->geometry.cg_to_rear_axle);

    fprintf(f, "[suspension]\n");
    fprintf(f, "spring_rate_fl     = %.2f\n", (double)params->suspension.spring_rate[0]);
    fprintf(f, "spring_rate_fr     = %.2f\n", (double)params->suspension.spring_rate[1]);
    fprintf(f, "spring_rate_rl     = %.2f\n", (double)params->suspension.spring_rate[2]);
    fprintf(f, "spring_rate_rr     = %.2f\n", (double)params->suspension.spring_rate[3]);
    fprintf(f, "damper_coeff_fl    = %.2f\n", (double)params->suspension.damper_coeff[0]);
    fprintf(f, "damper_coeff_fr    = %.2f\n", (double)params->suspension.damper_coeff[1]);
    fprintf(f, "damper_coeff_rl    = %.2f\n", (double)params->suspension.damper_coeff[2]);
    fprintf(f, "damper_coeff_rr    = %.2f\n", (double)params->suspension.damper_coeff[3]);
    fprintf(f, "rest_length_fl     = %.4f\n", (double)params->suspension.rest_length[0]);
    fprintf(f, "rest_length_fr     = %.4f\n", (double)params->suspension.rest_length[1]);
    fprintf(f, "rest_length_rl     = %.4f\n", (double)params->suspension.rest_length[2]);
    fprintf(f, "rest_length_rr     = %.4f\n", (double)params->suspension.rest_length[3]);
    fprintf(f, "roll_stiffness_front = %.2f\n", (double)params->suspension.roll_stiffness_front);
    fprintf(f, "roll_stiffness_rear  = %.2f\n\n", (double)params->suspension.roll_stiffness_rear);

    fprintf(f, "[wheel]\n");
    fprintf(f, "radius             = %.4f\n", (double)params->wheel.radius);
    fprintf(f, "width              = %.4f\n", (double)params->wheel.width);
    fprintf(f, "inertia            = %.4f\n", (double)params->wheel.inertia);
    fprintf(f, "rolling_resistance = %.4f\n\n", (double)params->wheel.rolling_resistance);

    fprintf(f, "[aero]\n");
    fprintf(f, "drag_coeff         = %.4f\n", (double)params->aero.drag_coeff);
    fprintf(f, "lift_coeff         = %.4f\n", (double)params->aero.lift_coeff);
    fprintf(f, "frontal_area       = %.4f\n", (double)params->aero.frontal_area);
    fprintf(f, "aero_balance       = %.4f\n", (double)params->aero.aero_balance);
    fprintf(f, "air_density        = %.4f\n\n", (double)params->aero.air_density);

    fprintf(f, "[brakes]\n");
    fprintf(f, "max_torque_front   = %.2f\n", (double)params->brakes.max_torque_front);
    fprintf(f, "max_torque_rear    = %.2f\n", (double)params->brakes.max_torque_rear);
    fprintf(f, "brake_balance      = %.4f\n\n", (double)params->brakes.brake_balance);

    fprintf(f, "[steering]\n");
    fprintf(f, "ratio              = %.4f\n", (double)params->steering.ratio);
    fprintf(f, "max_angle          = %.4f\n", (double)params->steering.max_angle);
    fprintf(f, "ackermann_factor   = %.4f\n\n", (double)params->steering.ackermann_factor);

    fprintf(f, "[driveline]\n");
    fprintf(f, "drive_config           = %d\n", (int)params->driveline.drive_config);
    fprintf(f, "engine_max_torque      = %.4f\n", (double)params->driveline.engine_max_torque);
    fprintf(f, "gear_ratio_1           = %.4f\n", (double)params->driveline.gear_ratios[0]);
    fprintf(f, "gear_ratio_2           = %.4f\n", (double)params->driveline.gear_ratios[1]);
    fprintf(f, "gear_ratio_3           = %.4f\n", (double)params->driveline.gear_ratios[2]);
    fprintf(f, "gear_ratio_4           = %.4f\n", (double)params->driveline.gear_ratios[3]);
    fprintf(f, "gear_ratio_5           = %.4f\n", (double)params->driveline.gear_ratios[4]);
    fprintf(f, "gear_ratio_6           = %.4f\n", (double)params->driveline.gear_ratios[5]);
    fprintf(f, "gear_ratio_r           = %.4f\n", (double)params->driveline.gear_ratios[6]);
    fprintf(f, "final_drive_ratio      = %.4f\n", (double)params->driveline.final_drive_ratio);
    fprintf(f, "drivetrain_efficiency  = %.4f\n\n", (double)params->driveline.drivetrain_efficiency);

    fprintf(f, "[tire_model]\n");
    fprintf(f, "# Pacejka Magic Formula — Hoosier FSAE slick\n");
    fprintf(f, "# Longitudinal (b-series)\n");
    fprintf(f, "b0  = %.4f\n", (double)params->tire_model.b0);
    fprintf(f, "b1  = %.4f\n", (double)params->tire_model.b1);
    fprintf(f, "b2  = %.4f\n", (double)params->tire_model.b2);
    fprintf(f, "b3  = %.4f\n", (double)params->tire_model.b3);
    fprintf(f, "b4  = %.4f\n", (double)params->tire_model.b4);
    fprintf(f, "b5  = %.4f\n", (double)params->tire_model.b5);
    fprintf(f, "b6  = %.4f\n", (double)params->tire_model.b6);
    fprintf(f, "b7  = %.4f\n", (double)params->tire_model.b7);
    fprintf(f, "b8  = %.4f\n", (double)params->tire_model.b8);
    fprintf(f, "b9  = %.4f\n", (double)params->tire_model.b9);
    fprintf(f, "b10 = %.4f\n", (double)params->tire_model.b10);
    fprintf(f, "# Lateral (a-series)\n");
    fprintf(f, "a0  = %.4f\n", (double)params->tire_model.a0);
    fprintf(f, "a1  = %.4f\n", (double)params->tire_model.a1);
    fprintf(f, "a2  = %.4f\n", (double)params->tire_model.a2);
    fprintf(f, "a3  = %.4f\n", (double)params->tire_model.a3);
    fprintf(f, "a4  = %.4f\n", (double)params->tire_model.a4);
    fprintf(f, "a5  = %.4f\n", (double)params->tire_model.a5);
    fprintf(f, "a6  = %.4f\n", (double)params->tire_model.a6);
    fprintf(f, "a7  = %.4f\n", (double)params->tire_model.a7);
    fprintf(f, "a8  = %.4f\n", (double)params->tire_model.a8);
    fprintf(f, "vertical_stiffness = %.2f\n", (double)params->tire_model.vertical_stiffness);

    fclose(f);
    return 0;
}

//-------------------------
// Apply to Components
// Each function performs a one-time copy of parameter values into a
// component's private fields via that component's public setter API.
// After these calls return, the VehicleParameters struct is no longer
// referenced by any component — simulation has zero dependency on it.
//-------------------------

/**
 * Apply suspension parameters to all four corners.
 * Guiggiani Sec. 3.8: F_spring = k·Δz, F_damper = c·Δ·ż
 */
void vehicle_params_apply_to_suspension(const VehicleParameters* p, Suspension* s) {
    if (!p || !s) return;
    for (int i = 0; i < 4; i++) {
        SuspensionCorner corner;
        suspension_get_corner(s, i, &corner);
        corner.spring_rate       = p->suspension.spring_rate[i];
        corner.damper_coeff      = p->suspension.damper_coeff[i];
        corner.rest_length       = p->suspension.rest_length[i];
        corner.spring_deflection = (vde_real)0.0;
        corner.damper_velocity   = (vde_real)0.0;
        suspension_set_corner(s, i, &corner);
    }
}

/**
 * Apply wheel geometry and inertia to all four wheel objects.
 * wheels[4] may contain NULLs — each is checked individually.
 */
void vehicle_params_apply_to_wheels(const VehicleParameters* p, Wheel* wheels[4]) {
    if (!p || !wheels) return;
    for (int i = 0; i < 4; i++) {
        if (!wheels[i]) continue;
        wheel_set_radius(wheels[i], p->wheel.radius);
        wheel_set_inertia(wheels[i], p->wheel.inertia);
    }
}

/**
 * Apply brake torque limits and front/rear balance.
 * Guiggiani Sec. 4.6: optimal φ = c/L + μ·h/L
 */
void vehicle_params_apply_to_brakes(const VehicleParameters* p, Brakes* b) {
    if (!p || !b) return;
    brakes_set_max_torque_front(b, p->brakes.max_torque_front);
    brakes_set_max_torque_rear(b, p->brakes.max_torque_rear);
    brakes_set_brake_balance(b, p->brakes.brake_balance);
}

/**
 * Apply steering geometry to the steering component.
 * Guiggiani Sec. 3.2.3: Ackermann geometry for kinematic correctness.
 */
void vehicle_params_apply_to_steering(const VehicleParameters* p, Steering* s) {
    if (!p || !s) return;
    steering_set_ratio(s, p->steering.ratio);
    steering_set_max_angle(s, p->steering.max_angle);
    steering_set_ackermann_factor(s, p->steering.ackermann_factor);
    // Pass chassis geometry so Ackermann computation has correct dimensions
    steering_set_wheelbase(s, p->geometry.wheelbase);
    steering_set_track_width(s, (p->geometry.track_front + p->geometry.track_rear) * (vde_real)0.5);
}

/**
 * Apply driveline configuration, gear ratios, and efficiency.
 * Guiggiani Sec. 3.11.4: differential mechanism principles.
 */
void vehicle_params_apply_to_driveline(const VehicleParameters* p, Driveline* d) {
    if (!p || !d) return;
    driveline_set_drive_config(d, (int)p->driveline.drive_config);
    driveline_set_engine_torque(d, p->driveline.engine_max_torque);
    driveline_set_gear_ratios(d, p->driveline.gear_ratios, 7);
    driveline_set_final_drive(d, p->driveline.final_drive_ratio);
    driveline_set_drivetrain_efficiency(d, p->driveline.drivetrain_efficiency);
}

/**
 * Apply aerodynamic coefficients, reference area, balance, and air density.
 * Guiggiani Sec. 7.6: F_drag = ½ρACd·V², F_down = ½ρACl·V²
 */
void vehicle_params_apply_to_aerodynamics(const VehicleParameters* p, Aerodynamics* a) {
    if (!p || !a) return;
    aerodynamics_set_drag_coeff(a, p->aero.drag_coeff);
    aerodynamics_set_lift_coeff(a, p->aero.lift_coeff);
    aerodynamics_set_frontal_area(a, p->aero.frontal_area);
    aerodynamics_set_aero_balance(a, p->aero.aero_balance);
    aerodynamics_set_air_density(a, p->aero.air_density);
}

/**
 * Set sprung-mass inertia tensor from the scalar roll/pitch/yaw values.
 * Off-diagonal terms are zero (symmetric vehicle assumption).
 * Guiggiani Sec. 3.10, Ch. 9.
 */
void vehicle_params_apply_to_sprung_mass(const VehicleParameters* p, SprungMass* sm) {
    if (!p || !sm) return;
    sprung_mass_set_mass(sm, p->mass.sprung_mass);

    // Build diagonal inertia tensor  [Ixx  0    0  ]
    //                                [0    Iyy  0  ]
    //                                [0    0    Izz]
    vde_mat3 I = vde_mat3_make(
        p->mass.inertia_roll,  (vde_real)0.0, (vde_real)0.0,
        (vde_real)0.0, p->mass.inertia_pitch,  (vde_real)0.0,
        (vde_real)0.0, (vde_real)0.0, p->mass.inertia_yaw
    );
    sprung_mass_set_inertia(sm, &I);
}

/**
 * Copy tire model coefficients into a MagicFormulaTireModel.
 * VehicleTireModelParams mirrors MagicFormulaParams field-for-field
 * so this is essentially a typed memcpy.
 * Guiggiani Sec. 2.10: y(x) = D·sin[C·arctan{B·x − E·(B·x − arctan(B·x))}]
 */
void vehicle_params_apply_to_tire_model(const VehicleParameters* p,
                                        MagicFormulaTireModel* mf) {
    if (!p || !mf) return;
    MagicFormulaParams mfp;
    mfp.b0  = p->tire_model.b0;
    mfp.b1  = p->tire_model.b1;
    mfp.b2  = p->tire_model.b2;
    mfp.b3  = p->tire_model.b3;
    mfp.b4  = p->tire_model.b4;
    mfp.b5  = p->tire_model.b5;
    mfp.b6  = p->tire_model.b6;
    mfp.b7  = p->tire_model.b7;
    mfp.b8  = p->tire_model.b8;
    mfp.b9  = p->tire_model.b9;
    mfp.b10 = p->tire_model.b10;
    mfp.a0  = p->tire_model.a0;
    mfp.a1  = p->tire_model.a1;
    mfp.a2  = p->tire_model.a2;
    mfp.a3  = p->tire_model.a3;
    mfp.a4  = p->tire_model.a4;
    mfp.a5  = p->tire_model.a5;
    mfp.a6  = p->tire_model.a6;
    mfp.a7  = p->tire_model.a7;
    mfp.a8  = p->tire_model.a8;
    mfp.vertical_stiffness = p->tire_model.vertical_stiffness;
    magic_formula_set_params(mf, &mfp);
}
