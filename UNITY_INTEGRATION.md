# Unity DLL Integration Architecture

**Complete guide to the FullCarSim ↔ Unity interface**

---

## 🎯 Overview

FullCarSim is built as a **native DLL** that Unity loads to power high-fidelity vehicle physics. The architecture separates concerns clearly:

```
┌─────────────────────────────────────────────────────────────┐
│                         UNITY (C#)                          │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │  Rendering  │  │ Input System │  │ Telemetry/UI     │  │
│  └──────┬──────┘  └──────┬───────┘  └────────┬─────────┘  │
│         │                │                    │             │
│         └────────────────┼────────────────────┘             │
│                          │                                  │
└──────────────────────────┼──────────────────────────────────┘
                           │ DLL Boundary (unity_api.h)
┌──────────────────────────┼──────────────────────────────────┐
│                          ▼                                  │
│                  ┌───────────────┐                          │
│                  │   Unity API   │  ◄── unity_api.h         │
│                  └───────┬───────┘                          │
│                          │                                  │
│         ┌────────────────┼────────────────┐                 │
│         ▼                ▼                ▼                 │
│   ┌──────────┐  ┌──────────────┐  ┌────────────┐          │
│   │ Vehicle  │  │  Simulation  │  │ Telemetry  │          │
│   │  Model   │  │     Loop     │  │  System    │          │
│   └────┬─────┘  └──────┬───────┘  └─────┬──────┘          │
│        │               │                 │                 │
│   ┌────▼──────────┐   │            ┌────▼────────┐        │
│   │ Congruence    │   │            │  Assets     │        │
│   │ Constitutive  │   │            │  (Maps,     │        │
│   │ Equilibrium   │   │            │   Configs)  │        │
│   └───────────────┘   │            └─────────────┘        │
│                       │                                     │
│   ┌───────────────────▼──────────────────────┐            │
│   │  Physics Core (Guiggiani Implementation) │            │
│   │  • Tires (Magic Formula, Brush)          │            │
│   │  • Dynamics Solver                       │            │
│   │  • Integration (RK4, Euler)              │            │
│   │  • Math Library (vec3, quat, frames)     │            │
│   └──────────────────────────────────────────┘            │
│                                                             │
│                    NATIVE DLL (C11)                        │
└─────────────────────────────────────────────────────────────┘
```

---

## 📋 DLL API Structure (unity_api.h)

The Unity API is organized into **8 functional groups**:

### 1. **Lifecycle Management**
```c
VehicleSim_Create()         // Create simulation instance
VehicleSim_Destroy()        // Cleanup
VehicleSim_LoadVehicle()    // Load vehicle config from file
VehicleSim_LoadTrack()      // Load track/map from file
VehicleSim_Initialize()     // Initialize after loading assets
VehicleSim_Reset()          // Reset to initial state
```

### 2. **Simulation Control**
```c
VehicleSim_Step()           // Advance by one physics timestep
VehicleSim_StepMultiple()   // Advance by N timesteps
VehicleSim_GetTime()        // Get current simulation time
```

### 3. **Input Interface** (Unity → DLL)
```c
VehicleSim_SetInputs()      // Full input struct
VehicleSim_SetBasicInputs() // Simple: throttle, brake, steering
```

Input structure:
```c
typedef struct DriverInputs {
    double throttle;    // [0, 1]
    double brake;       // [0, 1]
    double steering;    // [-1, 1]
    double clutch;      // [0, 1]
    int gear;           // -1=reverse, 0=neutral, 1+=forward
} DriverInputs;
```

### 4. **Rendering Data** (DLL → Unity) ⭐ PRIMARY INTERFACE
```c
VehicleSim_GetRenderData()  // Get all data needed for visualization
```

Returns comprehensive `VehicleRenderData`:
- **Chassis:** Position, orientation (quaternion), velocities
- **Wheels (×4):** Position, rotation, steer angle, suspension, tire forces
- **Visual feedback:** Speed, RPM, gear, steering wheel angle
- **Effects data:** Tire slip (for smoke), aero forces

### 5. **Telemetry Output** (DLL → Unity/File)
```c
VehicleSim_RegisterTelemetryCallback()  // Register callback for live data
VehicleSim_GetTelemetry()               // Get latest telemetry frame
VehicleSim_StartTelemetryRecording()    // Start buffering
VehicleSim_ExportTelemetryCSV()         // Export to file
```

`TelemetryFrame` includes:
- Complete vehicle state (position, velocity, acceleration)
- All tire data (slip, forces, loads)
- Suspension data
- Load transfers (Guiggiani Section 3.7)
- Driveline state
- Energy metrics

### 6. **Asset Management**
```c
VehicleSim_GetVehicleParameters()  // Read loaded vehicle config
VehicleSim_GetTrackInfo()          // Read loaded track info
VehicleSim_QuerySurface()          // Query track surface properties
```

### 7. **Error Handling**
```c
VehicleSim_GetLastError()
VehicleSim_ClearError()
```

### 8. **Debugging & Validation**
```c
VehicleSim_Validate()    // Check for NaN, unreasonable values
VehicleSim_GetStats()    // Performance statistics
```

---

## 🔄 Typical Unity Integration Flow

### Initialization (Unity Start)
```csharp
public class VehicleController : MonoBehaviour 
{
    private IntPtr simHandle;
    
    void Start() {
        // 1. Create simulation (1 ms physics timestep)
        simHandle = VehicleSim_Create(0.001);
        
        // 2. Load assets
        int result = VehicleSim_LoadVehicle(simHandle, 
            "data/vehicles/TBReCar.txt");
        if (result != 0) {
            Debug.LogError("Failed to load vehicle");
            return;
        }
        
        result = VehicleSim_LoadTrack(simHandle, 
            "data/tracks/skidpad.txt");
        if (result != 0) {
            Debug.LogError("Failed to load track");
            return;
        }
        
        // 3. Initialize simulation
        result = VehicleSim_Initialize(simHandle);
        if (result != 0) {
            Debug.LogError("Failed to initialize simulation");
            return;
        }
        
        Debug.Log("Vehicle simulation ready!");
    }
}
```

### Physics Update (Unity FixedUpdate)
```csharp
void FixedUpdate() {
    // 1. Get player input
    float throttle = Input.GetAxis("Throttle");  // [0, 1]
    float brake = Input.GetAxis("Brake");        // [0, 1]
    float steering = Input.GetAxis("Horizontal"); // [-1, 1]
    
    // 2. Send inputs to simulation
    VehicleSim_SetBasicInputs(simHandle, throttle, brake, steering);
    
    // 3. Step physics
    VehicleSim_Step(simHandle);
}
```

### Rendering Update (Unity Update)
```csharp
void Update() {
    // 1. Get render data from DLL
    VehicleRenderData renderData;
    VehicleSim_GetRenderData(simHandle, out renderData);
    
    // 2. Update chassis transform
    transform.position = new Vector3(
        (float)renderData.chassis.position_x,
        (float)renderData.chassis.position_y,
        (float)renderData.chassis.position_z
    );
    
    transform.rotation = new Quaternion(
        (float)renderData.chassis.orientation_x,
        (float)renderData.chassis.orientation_y,
        (float)renderData.chassis.orientation_z,
        (float)renderData.chassis.orientation_w
    );
    
    // 3. Update each wheel
    for (int i = 0; i < 4; i++) {
        UpdateWheel(i, renderData.wheels[i]);
    }
    
    // 4. Update UI
    speedometer.SetSpeed(renderData.speed_kmh);
    tachometer.SetRPM(renderData.rpm);
    gearDisplay.SetGear(renderData.current_gear);
    
    // 5. Particle effects (tire smoke, dust)
    UpdateTireEffects(renderData.wheels);
}

void UpdateWheel(int index, WheelRenderData wheelData) {
    Transform wheelTransform = wheels[index];
    
    // Position
    wheelTransform.position = new Vector3(
        (float)wheelData.position_x,
        (float)wheelData.position_y,
        (float)wheelData.position_z
    );
    
    // Rotation (spin)
    wheelTransform.localRotation = Quaternion.Euler(
        Mathf.Rad2Deg * (float)wheelData.rotation_angle,
        Mathf.Rad2Deg * (float)wheelData.steer_angle,
        0f
    );
    
    // Suspension visual
    suspensionStruts[index].localPosition = 
        Vector3.up * (float)wheelData.suspension_deflection;
    
    // Tire smoke if slipping
    if (Mathf.Abs((float)wheelData.lateral_slip) > 0.1f ||
        Mathf.Abs((float)wheelData.longitudinal_slip) > 0.15f) {
        tireSmokeParticles[index].Play();
    }
}
```

### Telemetry Collection
```csharp
// Option 1: Callback (real-time)
void Start() {
    // Register callback for live telemetry
    TelemetryCallback callback = OnTelemetryFrame;
    VehicleSim_RegisterTelemetryCallback(simHandle, callback, IntPtr.Zero);
}

void OnTelemetryFrame(TelemetryFrame frame, IntPtr userData) {
    // Process telemetry (send to graph, log, etc.)
    Debug.Log($"Speed: {frame.speed} m/s, Lat accel: {frame.lateral_acceleration}");
}

// Option 2: Export to file
void OnApplicationQuit() {
    VehicleSim_ExportTelemetryCSV(simHandle, "telemetry/session_" + 
        DateTime.Now.ToString("yyyyMMdd_HHmmss") + ".csv");
}
```

---

## 📁 Asset Management Architecture

### Directory Structure
```
StreamingAssets/
├── data/
│   ├── vehicles/          ← Vehicle configuration files
│   │   ├── TBReCar.txt
│   │   ├── FormulaE.txt
│   │   └── RoadCar.txt
│   │
│   ├── tracks/            ← Track definition files
│   │   ├── skidpad.txt
│   │   ├── figure8.txt
│   │   └── spa_francorchamps.txt
│   │
│   └── configs/           ← Simulation configs
│       └── config1.txt
```

### Asset Loading Flow

1. **Unity decides what to load** based on scene/menu selection
2. **Unity passes file paths** to DLL via LoadVehicle/LoadTrack
3. **DLL reads files**, parses, validates
4. **DLL stores** parsed data internally
5. **Unity can query** loaded data via GetVehicleParameters/GetTrackInfo

### Vehicle Config File Format (Example)
```
# TBReCar Vehicle Parameters
# Guiggiani-style parameter set

[Mass Properties]
mass = 250.0              # kg
wheelbase = 1.600         # m
track_front = 1.200       # m
track_rear = 1.200        # m
cg_height = 0.280         # m
cg_to_front = 0.800       # m
cg_to_rear = 0.800        # m

[Inertia]
inertia_roll = 50.0       # kg·m²
inertia_pitch = 120.0     # kg·m²
inertia_yaw = 120.0       # kg·m²

[Tires]
tire_radius = 0.280       # m
# Magic Formula parameters...
b0 = 1.5
b1 = 0.0
# ...

[Suspension]
spring_rate_front = 30000 # N/m
spring_rate_rear = 30000  # N/m
damping_front = 2000      # N·s/m
damping_rear = 2000       # N·s/m

[Aerodynamics]
drag_coefficient = 0.8
lift_coefficient = -2.5   # negative = downforce
frontal_area = 1.2        # m²

[Engine]
max_torque = 150          # Nm
max_rpm = 12000
```

### Track Config File Format (Example)
```
# Skidpad Track

[Info]
name = "Formula SAE Skidpad"
length = 157.08           # m (50m diameter circles)

[Start]
position = 0.0, 0.0, 0.0  # x, y, z
orientation = 0.0, 0.0, 0.0, 1.0  # quaternion (w,x,y,z)

[Surface]
friction = 1.0            # dry asphalt
# More detailed surface data...
```

---

## 📊 Telemetry System

### Purpose
Generate **comprehensive datasets** for:
- Setup optimization
- Driver analysis
- Machine learning training data
- Performance validation
- Debugging

### Data Collection Modes

#### 1. Live Callback (Real-time)
```csharp
VehicleSim_RegisterTelemetryCallback(simHandle, callback, userData);
```
- Called every physics step
- Low latency
- For real-time graphs, HUD

#### 2. Buffered Recording
```csharp
VehicleSim_StartTelemetryRecording(simHandle);
// ... drive around ...
VehicleSim_ExportTelemetryCSV(simHandle, "output.csv");
```
- Stores all frames in memory
- Export to CSV at end
- For post-session analysis

#### 3. Query Latest
```csharp
TelemetryFrame frame;
VehicleSim_GetTelemetry(simHandle, out frame);
```
- Get single latest frame
- For HUD updates

### Telemetry Frame Contents (73+ channels)

**Time:** time, delta_time

**Position/Orientation:** 
- position[3], velocity[3], acceleration[3]
- orientation[4] (quaternion), angular_velocity[3], angular_acceleration[3]

**Derived quantities:**
- speed, yaw_rate, lateral_accel, longitudinal_accel

**Inputs:**
- throttle, brake, steering, gear

**Tire data (×4 wheels = 16 channels):**
- slip_ratio, slip_angle, vertical_load, forces (long/lat/vert)

**Suspension (×4 = 12 channels):**
- deflection, velocity, force

**Load transfers (3 channels):**
- longitudinal, lateral_front, lateral_rear

**Driveline:**
- engine_rpm, engine_torque, wheel_angular_vel[4]

**Aerodynamics:**
- drag, downforce, side_force

**Energy:**
- kinetic_energy, power_output

### CSV Export Format
```csv
time,position_x,position_y,position_z,velocity_x,velocity_y,velocity_z,...
0.000,0.0,0.0,0.0,0.0,0.0,0.0,...
0.001,0.001,0.0,0.0,0.1,0.0,0.0,...
...
```

---

## 🔍 Surface Query System

Unity terrain can interact with DLL physics:

```csharp
// Unity queries track surface at wheel positions
SurfaceQuery query;
query.position[0] = wheelPos.x;
query.position[1] = wheelPos.y;
query.position[2] = wheelPos.z;

VehicleSim_QuerySurface(simHandle, ref query);

if (query.is_valid == 1) {
    float friction = (float)query.friction_coefficient;
    Vector3 normal = new Vector3(
        (float)query.normal[0],
        (float)query.normal[1],
        (float)query.normal[2]
    );
    float elevation = (float)query.elevation;
    
    // Use for Unity terrain mesh, grass effects, etc.
}
```

---

## ⚙️ Performance Considerations

### Physics Rate vs Render Rate

**Physics:** Fixed timestep (e.g., 1000 Hz = 0.001s)
- Called in `FixedUpdate`
- Deterministic
- High frequency for stability

**Rendering:** Variable (e.g., 60 Hz)
- Called in `Update`
- Interpolate if needed
- Lower frequency OK

### Multi-stepping
If simulation falls behind:
```csharp
void FixedUpdate() {
    int stepsNeeded = CalculateStepsNeeded();
    VehicleSim_StepMultiple(simHandle, stepsNeeded);
}
```

### Memory Considerations
- Render data: ~1 KB per frame (lightweight)
- Telemetry: ~600 bytes per frame
- 1 minute at 1000 Hz = ~35 MB telemetry

---

## 🛡️ Error Handling

### Validation
```csharp
ValidationResult result;
VehicleSim_Validate(simHandle, out result);

if (result.is_valid == 0) {
    Debug.LogError($"Simulation invalid: {result.message}");
    VehicleSim_Reset(simHandle);
}
```

### Error Messages
```csharp
string error = Marshal.PtrToStringAnsi(
    VehicleSim_GetLastError(simHandle));
if (error != null) {
    Debug.LogError($"DLL Error: {error}");
}
```

---

## 📈 Debugging & Analysis

### Performance Stats
```csharp
SimulationStats stats;
VehicleSim_GetStats(simHandle, out stats);

Debug.Log($"Avg step: {stats.average_step_time_ms:F3} ms");
Debug.Log($"Real-time factor: {stats.real_time_factor:F2}x");
```

### Visual Debug Draws
```csharp
void OnDrawGizmos() {
    VehicleRenderData data;
    VehicleSim_GetRenderData(simHandle, out data);
    
    // Draw tire forces as arrows
    for (int i = 0; i < 4; i++) {
        Vector3 forceDir = new Vector3(
            (float)data.wheels[i].tire_force_x,
            (float)data.wheels[i].tire_force_y,
            (float)data.wheels[i].tire_force_z
        );
        Gizmos.DrawRay(wheelPositions[i], forceDir * 0.001f);
    }
}
```

---

## 🎓 Summary: Data Flow

```
Unity Input → DLL → Physics → DLL → Unity Rendering
     ↓                              ↓
  Steering                       Position
  Throttle                       Orientation  
  Brake                          Wheel states
                                 Visual data
                                     ↓
                                Unity Scene
                                     ↓
                             Player sees result
```

**Key insight:** Unity doesn't know about Guiggiani, tire slips, or equations of motion. It just sends inputs, gets render data, and displays it. All the physics complexity is hidden behind the DLL boundary.
