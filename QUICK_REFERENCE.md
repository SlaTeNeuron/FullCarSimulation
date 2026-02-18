# FullCarSim - Quick Architecture Reference

**One-page overview of the complete system**

---

## 🎯 What Is This?

**FullCarSim** is a high-fidelity vehicle dynamics simulation engine:
- Written in **C11**
- Built as a **native DLL for Unity**
- Based on **Guiggiani's "The Science of Vehicle Dynamics"**
- Purpose: Power realistic racing simulations

---

## 🏗️ Two-Layer Architecture

```
┌─────────────────────────────────────────────────────┐
│                   LAYER 1: Unity API                │
│                 (Simple, Stable, Exported)          │
│                                                     │
│  File: unity_api.h                                  │
│  Purpose: DLL boundary between C# and C            │
│                                                     │
│  Data In:  DriverInputs (throttle, brake, steer)   │
│  Data Out: VehicleRenderData (positions, wheels)   │
│            TelemetryFrame (73+ channels)           │
│                                                     │
│  Functions: VehicleSim_Create, _Step,              │
│             _GetRenderData, _GetTelemetry, etc.    │
└─────────────────────────────────────────────────────┘
                        ↕
┌─────────────────────────────────────────────────────┐
│              LAYER 2: Internal Physics              │
│             (Complex, Accurate, Private)            │
│                                                     │
│  Philosophy: Guiggiani's Three-Equation Structure   │
│                                                     │
│  1. CONGRUENCE (vehicle_congruence.h)              │
│     → Tire slips from motion                       │
│                                                     │
│  2. CONSTITUTIVE (vehicle_constitutive.h)          │
│     → Forces from slips, deflections               │
│                                                     │
│  3. EQUILIBRIUM (vehicle_equilibrium.h)            │
│     → F=ma, load transfers, integrate              │
│                                                     │
│  Components: Tires, Suspension, Chassis,           │
│              Steering, Brakes, Driveline           │
└─────────────────────────────────────────────────────┘
```

---

## 📊 Data Flow

### Typical Frame
```
1. Unity Update/FixedUpdate
   ↓
2. Get player input (steering wheel, pedals)
   ↓
3. Prepare DriverInputs structure with throttle, brake, steering
   ↓ [DLL Boundary]
   ↓
4. VehicleSim_Step(sim, &inputs) executes:
   a. Apply inputs to vehicle
   b. Congruence: Compute tire slips
   c. Constitutive: Evaluate tire forces
   d. Equilibrium: Solve F=ma
   e. Integrate: Update state
   ↓
5. VehicleSim_GetRenderData(out renderData)
   ↓ [DLL Boundary]
   ↓
6. Unity updates GameObjects:
   - Vehicle position/rotation
   - Wheel positions/rotations
   - Tire smoke particles
   - HUD (speedometer, tachometer)
   - Audio (engine RPM)
```

---

## 📁 Key Files

### Unity Integration
| File | Purpose |
|------|---------|
| `unity_api.h` | ⭐ THE DLL interface - start here |
| `data/vehicles/*.txt` | Vehicle configuration files |
| `data/tracks/*.txt` | Track/map definition files |

### Guiggiani Physics
| File | Purpose |
|------|---------|
| `vehicle_model.h` | Master header - three-equation orchestration |
| `vehicle_congruence.h` | Kinematics (tire slips) |
| `vehicle_constitutive.h` | Component behavior (forces) |
| `vehicle_equilibrium.h` | Dynamics (F=ma) |
| `tire_models/magic_formula.h` | Tire forces (Pacejka) |

### Documentation
| File | Purpose |
|------|---------|
| `AI_CONTEXT.md` | Architecture, standards, and implementation status |
| `include/README.md` | Header organization |
| `include/vehicle/README.md` | Vehicle components |
| `include/tire_models/README.md` | Tire models (detailed) |
| `include/core/physics/README.md` | Physics solvers |

---

## 📦 Unity API at a Glance

### Lifecycle
```c
VehicleSim_Create(timestep)              // Create sim instance
VehicleSim_LoadVehicle("path/car.txt")   // Load vehicle config
VehicleSim_LoadTrack("path/track.txt")   // Load track
VehicleSim_Initialize()                  // Init after loading
VehicleSim_Step(sim, &inputs)            // Advance physics with inputs
VehicleSim_Destroy()                     // Cleanup
```

### Data Structures
```c
// INPUTS (Unity → DLL)
DriverInputs {
    throttle, brake, steering, clutch, gear
}

// OUTPUTS (DLL → Unity)
VehicleRenderData {
    ChassisRenderData: position, orientation, velocities
    WheelRenderData[4]: position, rotation, steering, suspension
    Visual: speed_kmh, rpm, gear, steering_wheel_angle
}

TelemetryFrame {
    73+ channels: position, velocity, acceleration,
                  tire data, suspension, load transfers,
                  driveline, aerodynamics, energy
}
```

---

## 🔬 Guiggiani Physics at a Glance

### Three-Equation Model (Section 3.12)

**1. CONGRUENCE (Kinematic)**
- Input: Vehicle state (position, velocity, orientation)
- Compute: Tire slips (σ, α, φ)
- Reference: Chapter 3, Section 3.2

**2. CONSTITUTIVE (Component Behavior)**
- Input: Tire slips, suspension deflections
- Compute: Forces and moments
- Includes: Magic Formula (Ch 2), springs, dampers
- Reference: Chapter 3, Section 3.3

**3. EQUILIBRIUM (Force Balance)**
- Input: All forces and moments
- Compute: Accelerations (F=ma, M=Iα)
- Includes: Load transfers (Section 3.7)
- Reference: Chapter 3, Sections 3.4-3.6

### Key Components

**Tires** (Chapter 2)
- Most important component
- Generates all grip forces
- Models: Magic Formula (empirical), Brush (physical)

**Suspension** (Section 3.8, Chapter 8)
- Springs: F = k·Δz
- Dampers: F = c·v
- Load transfers: longitudinal, lateral

**Chassis** (Section 3.10, Chapter 9)
- 6DOF rigid body
- Sprung mass dynamics
- Full 3D orientation (quaternions)

**Driveline** (Section 3.11.4, 6.1, 7.1)
- Engine, transmission
- Differential (open/locked)
- Torque distribution

---

## 🎮 Unity C# Usage Example

```csharp
using System.Runtime.InteropServices;

[StructLayout(LayoutKind.Sequential)]
public struct DriverInputs {
    public double throttle;
    public double brake;
    public double steering;
    public double clutch;
    public int gear;
}

[DllImport("racing_sim")]
private static extern IntPtr VehicleSim_Create(double timestep);

[DllImport("racing_sim")]
private static extern void VehicleSim_Step(IntPtr sim, ref DriverInputs inputs);

[DllImport("racing_sim")]
private static extern void VehicleSim_GetRenderData(
    IntPtr sim, out VehicleRenderData data);

private IntPtr simHandle;

void Start() {
    simHandle = VehicleSim_Create(0.001); // 1 ms timestep
    VehicleSim_LoadVehicle(simHandle, "data/vehicles/TBReCar.txt");
    VehicleSim_LoadTrack(simHandle, "data/tracks/skidpad.txt");
    VehicleSim_Initialize(simHandle);
}

void FixedUpdate() {
    DriverInputs inputs = new DriverInputs {
        throttle = Input.GetAxis("Throttle"),
        brake = Input.GetAxis("Brake"),
        steering = Input.GetAxis("Horizontal"),
        clutch = 0.0,
        gear = 1
    };
    
    VehicleSim_Step(simHandle, ref inputs);
}

void Update() {
    VehicleRenderData data;
    VehicleSim_GetRenderData(simHandle, out data);
    
    // Update vehicle GameObject
    transform.position = new Vector3(
        (float)data.chassis.position_x,
        (float)data.chassis.position_y,
        (float)data.chassis.position_z);
    
    transform.rotation = new Quaternion(
        (float)data.chassis.orientation_x,
        (float)data.chassis.orientation_y,
        (float)data.chassis.orientation_z,
        (float)data.chassis.orientation_w);
}
```

---

## 🚀 Getting Started

### As Unity Developer:
1. Read `unity_api.h` for all function signatures and documentation
2. See `README.md` for build instructions and C# interop example
3. Build DLL from Visual Studio project
4. Drop DLL into Unity `Assets/Plugins/`
5. Create C# wrapper (see example above)
6. Load vehicle/track configurations
7. Call Step() in FixedUpdate, GetRenderData() in Update

### As Physics Developer:
1. Read `AI_CONTEXT.md` (architecture, status, next priorities)
2. Understand Guiggiani's three-equation structure
4. Start with tire models (Magic Formula)
5. Implement vehicle components
6. Build up to complete vehicle model
7. Connect to Unity API

### As AI Assistant:
1. Read `AI_CONTEXT.md` (covers everything)
2. Understand two-layer architecture
3. Keep Unity API simple and stable
4. Follow Guiggiani rigorously for physics
5. Reference book chapter/sections in comments
6. Update documentation as you go

---

## 📚 Essential Reading Order

1. **This file** - Overall architecture
2. **AI_CONTEXT.md** - Architecture, standards, implementation status
3. **unity_api.h** - THE DLL interface
4. **vehicle_model.h** - Three-equation structure
5. **include/README.md** - Header directory guide

---

## 🎯 Key Takeaways

✅ **Two layers:** Simple Unity API + Complex Guiggiani physics  
✅ **Clear boundary:** unity_api.h is THE interface  
✅ **Simple in, rich out:** Basic inputs → Comprehensive outputs  
✅ **Well-structured:** Three-equation model + modular components  
✅ **Documented:** Every header references Guiggiani  
✅ **Practical:** Built for real Unity integration  
✅ **Dataset-ready:** 73+ telemetry channels for ML/analysis  

---

**Questions? Check the detailed docs listed above!**
