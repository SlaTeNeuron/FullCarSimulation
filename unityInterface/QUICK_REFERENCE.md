# Racing Sim Debug Interface - Quick Reference

## Building

```bash
# Windows - Easy build
build_debug.bat

# Manual build
cd unityInterface/build
cmake ..
cmake --build . --config Debug
```

## Running

```bash
# Run all tests
sim_debug.exe

# Run specific test (1=accel, 2=brake, 3=turn, 4=slalom)
sim_debug.exe -test 1

# Interactive mode
sim_debug.exe -interactive
sim_debug.exe -i

# Custom timestep (default 0.001 = 1 kHz, matches DLL rate)
sim_debug.exe -dt 0.001
sim_debug.exe -dt 0.01

# Custom asset paths
sim_debug.exe -vehicle ../../data/vehicles/TBReCar.txt
sim_debug.exe -track   ../../data/tracks/skidpad.txt

# Combine options
sim_debug.exe -dt 0.001 -test 3
sim_debug.exe -i -dt 0.001
```

## Interactive Commands

| Command | Description | Example |
|---------|-------------|---------|
| `step [N]` | Run N steps (default 1) | `step 100` |
| `input T B S [G]` | Set throttle brake steer [gear] | `input 1.0 0.0 0.5 1` |
| `reset` | Reset + re-initialize | `reset` |
| `state` | Telemetry state snapshot | `state` |
| `render` | `VehicleRenderData` (Unity frame output) | `render` |
| `telem` | Full `TelemetryFrame` dump | `telem` |
| `params` | Vehicle parameters | `params` |
| `validate` | Run validation check | `validate` |
| `stats` | Step count, timing, real-time factor | `stats` |
| `diag` | Full diagnostics to stderr | `diag` |
| `quit` | Exit | `quit` |

## Input Ranges

| Input | Range | Description |
|-------|-------|-------------|
| Throttle | 0.0 to 1.0 | 0 = no throttle, 1 = full throttle |
| Brake | 0.0 to 1.0 | 0 = no brake, 1 = full brake |
| Steer | -1.0 to 1.0 | -1 = full left, 0 = centre, 1 = full right |
| Gear | -1, 0, 1–6 | -1 = reverse, 0 = neutral, 1–6 = forward |

## Example Workflow

```bash
# 1. Build the debug executable
cd unityInterface
build_debug.bat

# 2. Run interactive mode
run_debug.bat -i

# 3. In interactive mode:
> input 1.0 0.0 0.0 1     # Full throttle, gear 1
> step 100                # Run 100 steps
> state                   # Check state
> input 0.5 0.0 0.8 1     # Half throttle, right turn
> step 200                # Run more steps
> render                  # Check render data (Unity frame)
> validate                # Confirm sim is healthy
> stats                   # Review step timing
> quit
```

## Output Redirect

```bash
# Save stdout to file (stderr log still visible in console)
sim_debug.exe > results.txt

# Save and view simultaneously (PowerShell)
.\sim_debug.exe | Tee-Object -FilePath results.txt

# Suppress log noise, keep test output
.\sim_debug.exe 2>$null > results.txt
```

## Files

- `build/Debug/sim_debug.exe` — debug executable (statically links `unity_api.c`)
- `build/racing_sim.dll` — Unity DLL (same `unity_api.c` compiled as shared library)

> Both targets compile `unity_api.c` with `BUILDING_DLL` defined — identical code path.
> The `simulation/` subtree (src and include) was fully removed (February 2026).

## Common Use Cases

### Test a specific maneuver
```bash
sim_debug.exe -i
> input 0.7 0.0 0.5 1
> step 500
> state
> render
```

### Quick regression check (all tests, 1 kHz)
```bash
sim_debug.exe -dt 0.001 > results.txt
```

### Investigate an initialization failure
```bash
sim_debug.exe -i
> diag
```

### Performance profiling
```bash
# Measure 1000 steps
sim_debug.exe -test 1
# Check execution time in output
```

### Verify physics changes
```bash
# 1. Modify vehicle.c or other physics source
build_debug.bat
# 2. Run test
sim_debug.exe -test 3
# 3. Compare with expected behavior
```

### Record telemetry
```bash
sim_debug.exe -test 1 > accel_data.csv
# Parse telemetry lines for analysis
```
