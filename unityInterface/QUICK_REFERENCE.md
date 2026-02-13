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

# Custom timestep (default 0.02)
sim_debug.exe -dt 0.01

# Combine options
sim_debug.exe -dt 0.01 -test 3
```

## Interactive Commands

| Command | Description | Example |
|---------|-------------|---------|
| `step N` | Run N simulation steps | `step 100` |
| `input T B S` | Set throttle, brake, steer (-1 to 1) | `input 1.0 0.0 0.5` |
| `start` | Start simulation | `start` |
| `stop` | Stop simulation | `stop` |
| `state` | Print current state | `state` |
| `telem` | Get telemetry data | `telem` |
| `quit` | Exit | `quit` |

## Input Ranges

| Input | Range | Description |
|-------|-------|-------------|
| Throttle | 0.0 to 1.0 | 0 = no throttle, 1 = full throttle |
| Brake | 0.0 to 1.0 | 0 = no brake, 1 = full brake |
| Steer | -1.0 to 1.0 | -1 = full left, 0 = center, 1 = full right |

## Example Workflow

```bash
# 1. Build the debug executable
cd unityInterface
build_debug.bat

# 2. Run interactive mode
run_debug.bat -i

# 3. In interactive mode:
> start
> input 1.0 0.0 0.0     # Full throttle
> step 100              # Run 100 steps
> state                 # Check state
> input 0.5 0.0 0.8     # Half throttle, right turn
> step 200              # Run more steps
> state                 # Check state again
> quit
```

## Output Redirect

```bash
# Save output to file
sim_debug.exe > results.txt

# Save and view simultaneously (PowerShell)
.\sim_debug.exe | Tee-Object -FilePath results.txt
```

## Files Created

- `sim_debug.exe` - Debug executable (in `build/Debug/`)
- `racing_sim.dll` - Unity DLL (in `build/`)

## Common Use Cases

### Test a specific maneuver
```bash
sim_debug.exe -i
> start
> input 0.7 0.0 0.5
> step 500
> state
```

### Performance profiling
```bash
# Measure 1000 steps
sim_debug.exe -test 1
# Check execution time in output
```

### Verify physics changes
```bash
# 1. Modify vehicle.c or sim.c
# 2. Rebuild
build_debug.bat
# 3. Run test
sim_debug.exe -test 3
# 4. Compare with expected behavior
```

### Record telemetry
```bash
sim_debug.exe -test 1 > accel_data.csv
# Parse telemetry lines for analysis
```
