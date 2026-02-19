# Debug Test Harness - Racing Simulation

The `sim_debug` executable drives the simulation **through the same `VehicleSim_*` Unity API
that `racing_sim.dll` exports** — `unity_api.c` is compiled statically into it.
Every code path exercised here is functionally identical to what Unity runs.

---

## Architecture

```
sim_debug.exe
├── debug_main.c          ← test scenarios + interactive REPL
└── unity_api.c (static)  ← exact same file compiled into racing_sim.dll
          ↓
    vehicle_model.c, vehicle.c, tire models, math, integrators …
```

Both targets share `CORE_SRC` in `CMakeLists.txt` and define `BUILDING_DLL`, so
`UNITY_API` expands identically in both. Bugs found here exist in the DLL; fixes
made here fix the DLL.

> **Note:** The old internal `Sim*` API (`sim_create`, `sim_step`, `sim_get_state`, …)
> and all `simulation/` source and header files were **fully removed** in February 2026.
> All debug testing now flows exclusively through `unity_api.c`.

---

## Quick Start

### Build (Windows — easy)
```bash
build_debug.bat
```

### Manual build
```bash
cd unityInterface
mkdir build
cd build
cmake ..
cmake --build . --config Debug
```

Output: `build/Debug/sim_debug.exe`

---

## Running

### Run all four tests (default)
```bash
sim_debug.exe
```

### Run one specific test
```bash
sim_debug.exe -test <1-4>
```

| # | Test |
|---|------|
| 1 | Straight-line acceleration (200 steps) |
| 2 | Braking — accelerate 100 steps then full brake (100 steps) |
| 3 | Turning circle — half throttle, full right steer (300 steps) |
| 4 | Slalom — alternating steer every 50 steps (300 steps) |

### Custom timestep
```bash
sim_debug.exe -dt 0.001    # 1 kHz (default — matches recommended DLL rate)
sim_debug.exe -dt 0.01     # 100 Hz
```

### Custom asset paths
```bash
sim_debug.exe -vehicle ../../data/vehicles/TBReCar.txt -track ../../data/tracks/skidpad.txt
```

(Default paths are relative to the executable's working directory.)

### Interactive mode
```bash
sim_debug.exe -interactive
sim_debug.exe -i
```

### Combined
```bash
sim_debug.exe -dt 0.001 -test 3
sim_debug.exe -i -vehicle path/to/car.txt
```

---

## Interactive Commands

| Command | Description | Example |
|---------|-------------|---------|
| `step [N]` | Run N physics steps (default 1) | `step 100` |
| `input T B S [G]` | Set throttle brake steer [gear] | `input 1.0 0.0 0.0 1` |
| `reset` | Reset + re-initialize simulation | `reset` |
| `state` | Telemetry state snapshot (position, velocity, yaw…) | `state` |
| `render` | `VehicleRenderData` — exact struct Unity reads each frame | `render` |
| `telem` | Full `TelemetryFrame` dump (orientation, accel, ang vel) | `telem` |
| `params` | Print loaded `VehicleParameters` | `params` |
| `validate` | Run `VehicleSim_Validate` check | `validate` |
| `stats` | `SimulationStats` (steps, avg/max time, real-time factor) | `stats` |
| `diag` | Full diagnostics dump to stderr | `diag` |
| `quit` / `exit` | Exit | `quit` |

**Input ranges:**

| Field | Range | Notes |
|-------|-------|-------|
| Throttle T | 0.0 → 1.0 | 1.0 = full throttle |
| Brake B | 0.0 → 1.0 | 1.0 = full brake |
| Steer S | -1.0 → +1.0 | −1 = full left, +1 = full right |
| Gear G | -1, 0, 1–6 | optional; defaults to 1 |

### Example interactive session
```
> input 1.0 0.0 0.0 1
  Inputs: thr=1.00  brk=0.00  str=+0.00  gear=1
> step 500
  Ran 500 step(s) (total: 500)
--- Step 500 | T=  0.500 s ---
  Pos  [R=  0.000  U=  0.000  F=  2.495] m
  Vel  [R=  0.000  U=  0.000  F=  4.990] m/s  |  Speed: 4.990 m/s (18.0 km/h)
  Yaw:   +0.00 deg  |  Yaw rate: +0.0000 rad/s
  Inputs: thr=1.00  brk=0.00  str=+0.00  gear=1
  Wheels ω FL/FR/RL/RR:  24.95 /  24.95 /  24.95 /  24.95 rad/s
  Susp  Δ FL/FR/RL/RR: +0.0000 / +0.0000 / +0.0000 / +0.0000 m
> render
  [RenderData] pos=(0.000, 0.000, 2.495)  q=(w=1.000 x=0.000 y=0.000 z=0.000)  18.0 km/h
> validate
  Validation: PASS (code=0) — Simulation state valid
> stats
  Steps=500  avg=0.0010 ms  max=0.0015 ms  RTF=1.00x
> quit
```

---

## Output Format

### State snapshot (periodic during tests / `state` command)
```
--- Step 200 | T=  0.200 s ---
  Pos  [R=  0.000  U=  0.000  F=  1.234] m     ← Unity space: X=right Y=up Z=forward
  Vel  [R=  0.000  U=  0.000  F=  3.456] m/s  |  Speed: 3.456 m/s (12.4 km/h)
  Yaw:  +12.34 deg  |  Yaw rate: +0.0523 rad/s
  Inputs: thr=0.50  brk=0.00  str=+1.00  gear=1
  Wheels ω FL/FR/RL/RR:  17.28 /  17.28 /  17.28 /  17.28 rad/s
  Susp  Δ FL/FR/RL/RR: +0.0000 / +0.0000 / +0.0000 / +0.0000 m
```

**Coordinate note:** Positions, velocities, and quaternions are in **Unity world space**
(X=right, Y=up, Z=forward). The DLL boundary converts from the internal physics convention
at every output call — what you see here is exactly what Unity receives.

### Telemetry callback (every 10th step → stdout)
```
[Telem #  10 | T=  0.010 s]  pos=(0.00,0.00,0.05)  spd= 0.10 m/s  thr=1.00  brk=0.00  str=+0.00
```

### Log callback (every message → stderr)
```
[INF] [T=0.000][INF] Simulation created. timestep=0.001000 s. ...
[WRN] [T=0.000][WRN] LoadVehicle: path='...' - ALPHA: using hardcoded TBRe defaults ...
[INF] [T=0.000][INF] Initialize: SUCCESS. ...
```

---

## Features

### Identical Unity API code path
`unity_api.c` is compiled directly into the exe — no wrapper, no translation layer.
A bug found here exists in the DLL; a fix here fixes the DLL.

### Real-time log callback
`VehicleSim_SetLogCallback` wired to stderr, mirroring the Unity `[MonoPInvokeCallback]`
pattern. Captures `DEBUG` → `ERROR` in real time, including messages from inside `Step`.

### Full `TelemetryFrame` (73+ channels)
`VehicleSim_RegisterTelemetryCallback` fires every `VehicleSim_Step` with a complete
`TelemetryFrame`: position, velocity, acceleration, orientation, angular velocity,
tire data, suspension, load transfers, driveline, aerodynamics, energy.

### `VehicleRenderData` exercised each scenario
`print_render_data()` calls `VehicleSim_GetRenderData` after each test — the same
call Unity makes every rendered frame.

### Validation + diagnostics
`VehicleSim_Validate` runs at startup and end-of-session.
`VehicleSim_GetDiagnostics` (via `diag` command) prints the same diagnostic blob
available to Unity via `VehicleSim_GetDiagnostics`.

---

## Extending Tests

Add scenarios directly to `debug_main.c` using the Unity API:

```c
static void test_my_scenario(VehicleSimulation* sim, int steps) {
    printf("\n========================================\n");
    printf("TEST: My Custom Scenario\n");
    printf("========================================\n");

    DriverInputs inputs = {0};
    inputs.throttle = 0.8;
    inputs.steering = 0.3;
    inputs.gear     = 1;

    for (int i = 0; i < steps; i++) {
        VehicleSim_Step(sim, &inputs);
        if (i % 50 == 0 || i == steps - 1) {
            print_state(sim, i);
        }
    }
    print_render_data(sim);
}
```

Call from `main()` in the `test_mode == 0` block, or add a new `-test N` case.

---

## Output Redirect

```bash
# Stdout to file; stderr log still appears in console
sim_debug.exe > results.txt

# Capture both (PowerShell)
.\sim_debug.exe *>&1 | Tee-Object -FilePath results.txt

# Suppress log noise, keep test output
.\sim_debug.exe 2>$null > results.txt
```

---

## Troubleshooting

### Build errors
- Ensure CMake 3.15+ and a C11 compiler (MSVC / MinGW / Clang) are in PATH.
- All source files are listed in `CMakeLists.txt` — check `CORE_SRC` if a link fails.
- Both targets define `BUILDING_DLL`; `STATIC_BUILD` is no longer used.

### Initialization failures
Run `sim_debug.exe -i` and type `diag` immediately — it dumps the full ring-buffer log,
init state, timestep, and last error. The same diagnostic text is available in Unity via
`VehicleSim_GetDiagnostics`.

### Stepping but nothing moves
Check the `initialized=` field in `diag` output. `VehicleSim_Step` is silently
rate-limited (and logs an error) if `initialized=0`.

### Debugger attach (MSVC)
```bash
devenv ..\build\Debug\sim_debug.exe
```

---

## Benefits

✓ **No Unity required** — test simulation logic independently  
✓ **Identical code path** — same `unity_api.c` as the DLL, no divergence  
✓ **Fast iteration** — compile and run in seconds, no Unity reload  
✓ **Real-time log stream** — `SimLogCallback` to stderr, same as Unity  
✓ **Full telemetry** — 73+ channel `TelemetryFrame` every step  
✓ **Render data exercised** — `VehicleRenderData` verified each scenario  
✓ **Interactive REPL** — step frame-by-frame, inspect any output struct  
✓ **Reproducible tests** — same inputs → same outputs  

---

## Development Notes

- `unity_api.c` compiles into both `racing_sim.dll` (SHARED) and `sim_debug.exe` (static).
  Both targets define `BUILDING_DLL`; `__declspec(dllexport)` on an executable symbol is harmless.
- `CORE_SRC` in `CMakeLists.txt` is shared between both targets — a source file added
  to `CORE_SRC` is automatically available in both builds.
- `src/simulation/` was removed (February 2026). The old internal `Sim*` API no longer
  exists. The `include/simulation/` headers are retained but have no compiled source.
- Workflow: edit `vehicle.c` or physics code → `build_debug.bat` → `sim_debug.exe` →
  verify behavior → the same fix is present in the DLL.


## Quick Start

### Option 1: Use the build script (Windows)
```bash
build_debug.bat
```

### Option 2: Manual build
```bash
cd unityInterface/build
cmake ..
cmake --build . --config Debug
```

The debug executable will be created at: `../build/Debug/sim_debug.exe`

## Running the Debug Executable

### Run All Tests (Default)
```bash
sim_debug.exe
```
Runs all predefined test scenarios:
- Test 1: Straight line acceleration
- Test 2: Braking behavior
- Test 3: Turning circle
- Test 4: Slalom pattern

### Run Specific Test
```bash
sim_debug.exe -test <1-4>
```
Examples:
- `sim_debug.exe -test 1` - Only run acceleration test
- `sim_debug.exe -test 2` - Only run braking test
- `sim_debug.exe -test 3` - Only run turning test
- `sim_debug.exe -test 4` - Only run slalom test

### Custom Timestep
```bash
sim_debug.exe -dt 0.01
```
Changes simulation timestep (default: 0.02 seconds = 50 Hz)

### Interactive Mode
```bash
sim_debug.exe -interactive
```
or
```bash
sim_debug.exe -i
```

Launches an interactive command prompt where you can:

#### Interactive Commands:
- `step N` - Run N simulation steps (default: 1)
- `input T B S` - Set throttle, brake, steer (each -1 to 1)
  - Example: `input 1.0 0.0 0.5` (full throttle, no brake, half right steer)
- `start` - Start the simulation
- `stop` - Stop the simulation
- `state` - Print current simulation state
- `telem` - Get latest telemetry data
- `quit` - Exit interactive mode

#### Example Interactive Session:
```
> start
Simulation started
> input 1.0 0.0 0.0
Set inputs: throttle=1.00, brake=0.00, steer=0.00
> step 100
Executed 100 steps (total: 100)
=== Step 100 | Time: 2.000 s ===
Position: (10.532, 0.000, 0.000) m
Velocity: 5.234 m/s
...
> input 0.0 1.0 0.0
Set inputs: throttle=0.00, brake=1.00, steer=0.00
> step 50
Executed 50 steps (total: 150)
=== Step 150 | Time: 3.000 s ===
Position: (15.832, 0.000, 0.000) m
Velocity: 2.134 m/s
...
> quit
```

## Features

### Automatic Telemetry Logging
- Telemetry callback fires every simulation step
- Prints summary every 10th frame to avoid spam
- Shows position, velocity, yaw, and other data

### State Inspection
Each test periodically prints comprehensive state information:
- Elapsed time
- Position (x, y, z)
- Orientation (yaw, pitch, roll in degrees)
- Velocity and acceleration
- Steering angle and motor RPM

### Predefined Test Scenarios
1. **Acceleration Test**: Full throttle for 200 steps to test motor response
2. **Braking Test**: Accelerate then brake to test deceleration behavior
3. **Turning Test**: Constant throttle with full steering to test circular motion
4. **Slalom Test**: Alternating steering to test dynamic response

## Benefits

✓ **No Unity Required**: Test simulation logic independently  
✓ **Fast Iteration**: Compile and run in seconds, no Unity reload  
✓ **Detailed Logging**: See telemetry and state data in console  
✓ **Reproducible Tests**: Same inputs produce same outputs  
✓ **Interactive Debugging**: Step through simulation frame by frame  
✓ **Easy Automation**: Script tests or integrate with CI/CD  

## Workflow Example

1. Make changes to simulation code (e.g., `vehicle.c`, `sim.c`)
2. Run `build_debug.bat` to rebuild
3. Run `sim_debug.exe -i` for interactive testing
4. Verify behavior before loading into Unity

## Troubleshooting

### Build Errors
- Ensure you have CMake installed and in PATH
- Make sure you have a C compiler (MSVC, MinGW, etc.)
- Check that all source files in `src/` and `include/` are present

### Link Errors
- If linking fails, check that all simulation source files are listed in CMakeLists.txt
- Ensure CORE_SRC includes all necessary .c files

### Runtime Errors
- If simulation crashes, run with Visual Studio debugger:
  ```
  devenv ../build/Debug/sim_debug.exe
  ```

## Extending Tests

To add your own test scenarios, edit `debug_main.c`:

```c
static void test_my_scenario(Sim* sim, int steps) {
    printf("\n========================================\n");
    printf("TEST: My Custom Test\n");
    printf("========================================\n");
    
    sim_start(sim);
    
    for (int i = 0; i < steps; i++) {
        // Set custom inputs
        sim_set_inputs(sim, custom_throttle, custom_brake, custom_steer);
        sim_step(sim);
        
        if (i % 50 == 0) {
            print_state(sim, i);
        }
    }
    
    sim_stop(sim);
}
```

Then call it from `main()`.

## Output Files

You can redirect output to a file for analysis:
```bash
sim_debug.exe > test_results.txt
```

Or in PowerShell:
```powershell
.\sim_debug.exe | Tee-Object -FilePath test_results.txt
```

## Development Notes

- The debug executable statically links simulation code (no DLL dependency)
- Both DLL and debug exe are built from the same source files
- Changes to simulation code automatically apply to both targets
- CMake handles separate compilation flags for each target
