# Debug Test Environment for Racing Simulation

This debug interface allows you to test and debug the simulation without needing to reload Unity.

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
