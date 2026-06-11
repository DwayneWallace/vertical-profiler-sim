# Vertical Profiler Simulator

## Overview

This project is a Python-based simulator for testing and tuning vertical profiler control systems.

The goal of this project is to make a simple and easy test bed for tuning PID loops on different vertical profilers. Instead of needing to test every controller change on physical hardware, the simulator allows different profiler designs to be modeled with approximate mass, volume, buoyancy engine size, actuator range, drag, water density, and mission settings.

The simulator is designed for early testing, controller tuning, and design comparison. It helps show how a vertical profiler responds to different PID gains, vehicle properties, actuator behavior, and target depths.

This is not meant to perfectly replace real pool testing. It is meant to make the tuning process easier before hardware testing.

## Purpose

Vertical profilers are hard to tune because they do not respond instantly. The buoyancy engine changes the vehicle’s displacement, but the profiler still has momentum. This can cause overshoot, oscillation, slow settling, or poor target holding.

This simulator was created to help answer questions such as:

* Why does the profiler overshoot the target depth?
* Why does the profiler oscillate even when the proportional gain is low?
* How much does drag affect stability?
* How much does vehicle mass affect control response?
* How does actuator delay affect PID tuning?
* What happens when the same PID values are used on different profiler designs?
* How does the mission behave when the profiler has limited buoyancy force?
* Would a different controller work better than a basic PID loop?

## Current Features

The current simulator includes:

* Multiple saved vertical profiler configurations
* Editable mission settings
* Editable PID settings
* Start, pause, reset, and apply controls
* Radio button selection between profiler designs
* Real-time depth graph
* Real-time actuator command graph
* Simple water-column depth graphic
* Target depth marker
* Target tolerance band
* Simulated measured depth with sensor noise
* Mission state machine
* PID control loop
* Buoyancy force calculation
* Drag force calculation
* Actuator command mapping
* Velocity and acceleration simulation

## Included Vehicle Profiles

The simulator currently includes three profiler configurations:

### LBCC 2026

A vertical profiler configuration using:

* Mass: 1.2 kg
* Water density: 1023 kg/m³
* Neutral volume: 0.001173 m³
* Reference area: 0.0070 m²
* Drag coefficient: 0.9
* Buoyancy engine delta: 19.6 ml
* Actuator range: 500–2500 µs
* Neutral command: 1500 µs

### Rays 2026

A similar current-generation profiler configuration with a different default PID gain and mission depth settings.

### Rays 2024

An earlier profiler configuration using:

* Mass: 1.8 kg
* Water density: 997 kg/m³
* Neutral volume: 0.001805 m³
* Reference area: 0.0080 m²
* Drag coefficient: 0.8
* Buoyancy engine delta: 26.0 ml
* Actuator range: 1000–1300 µs
* Neutral command: 1150 µs

## Mission Model

The simulator uses a mission state machine. The profiler moves through a sequence of target depths and hold states.

The current mission states are:

* `IDLE`
* `DESCEND_1`
* `HOLD_DEEP_1`
* `ASCEND_1`
* `HOLD_SHALLOW_1`
* `DESCEND_2`
* `HOLD_DEEP_2`
* `ASCEND_2`
* `HOLD_SHALLOW_2`
* `STATION_KEEP`
* `RECOVER_SURFACE`
* `DONE`

The mission moves between states based on:

* Whether the measured depth is inside the tolerance band
* How long the profiler has held its target
* Maximum transit time
* Maximum hold time
* Surface recovery timeout

The hold timer resets if the profiler leaves the target tolerance range.

## PID Control

The simulator uses a basic PID control loop to drive the buoyancy engine toward the current target depth.

The depth error is calculated as:

```text
error = target depth - measured depth
```

The PID output is calculated from:

```text
output = Kp × error + Ki × integral + Kd × derivative
```

The PID output is added to the actuator neutral command:

```text
actuator command = neutral command + PID output
```

The actuator command is then clamped to the allowed actuator range:

```text
min command ≤ actuator command ≤ max command
```

The simulator includes integral clamping to prevent the integral term from growing too large.

## Actuator Model

The simulator converts actuator microseconds into a normalized command value.

The actuator command is centered around the neutral command:

```text
centered command = actuator command - neutral command
```

The command is normalized by half of the actuator range:

```text
normalized command = centered command / half range
```

The command is then clamped between -1 and 1:

```text
-1 ≤ normalized command ≤ 1
```

A direction value is applied so the simulator can handle different actuator wiring or mechanical directions.

```text
final actuator command = direction × normalized command
```

This command is used to change the simulated buoyancy force.

## Buoyancy Model

The simulator calculates buoyancy force from displaced volume:

```text
buoyancy force = water density × gravity × displaced volume
```

The neutral buoyancy force is based on the profiler’s neutral volume:

```text
neutral buoyancy force = ρ × g × neutral volume
```

The buoyancy engine changes the effective displaced volume. In the simulator, the actuator command changes buoyancy around the neutral point:

```text
buoyancy force = neutral buoyancy force + command × maximum buoyancy delta
```

The maximum buoyancy delta is calculated from the buoyancy engine volume:

```text
maximum buoyancy delta = ρ × g × (buoyancy engine volume / 2)
```

This means the buoyancy engine can make the profiler more or less buoyant around the neutral condition.

## Weight Force

Weight force is calculated from the mass of the profiler:

```text
weight force = mass × gravity
```

Where:

```text
gravity = 9.81 m/s²
```

## Drag Model

The simulator uses a quadratic drag model:

```text
drag force = -0.5 × ρ × Cd × A × v × |v|
```

Where:

* `ρ` is water density
* `Cd` is drag coefficient
* `A` is reference area
* `v` is vertical velocity

The `v × |v|` term makes drag increase with the square of velocity while keeping the drag force opposite the direction of motion.

This is important because drag helps slow the profiler down and reduces overshoot at higher speeds.

## Net Force

The net force is calculated from weight, buoyancy, and drag:

```text
net force = weight force - buoyancy force + drag force
```

In this simulator, increasing depth is treated as downward motion. The sign convention is based on depth increasing as the profiler moves down.

If net force is positive, the profiler accelerates downward.

If net force is negative, the profiler accelerates upward.

## Acceleration

The simulator uses Newton’s second law:

```text
force = mass × acceleration
```

Solving for acceleration:

```text
acceleration = net force / mass
```

The simulator updates the profiler’s acceleration every time step based on the current force balance.

## Velocity Calculation

Velocity is updated by integrating acceleration over time:

```text
velocity = velocity + acceleration × dt
```

This is a simple numerical integration step.

The simulator uses this velocity to update depth:

```text
depth = depth + velocity × dt
```

This allows the profiler to keep moving even after the actuator changes command, which is important for modeling overshoot and oscillation.

## Handwritten Velocity Proof

The simulator’s velocity calculation was based on a handwritten proof and derivation.

### Velocity Proof Photo 1

Add image here:

```markdown
![Velocity proof page 1](docs/images/velocity-proof-1.jpg)
```

### Velocity Proof Photo 2

Add image here:

```markdown
![Velocity proof page 2](docs/images/velocity-proof-2.jpg)
```

Place the images in:

```text
docs/images/
```

Suggested file names:

```text
velocity-proof-1.jpg
velocity-proof-2.jpg
```

## Depth Calculation

Depth is updated from velocity:

```text
new depth = old depth + velocity × dt
```

Because the simulation treats depth as positive downward, positive velocity increases depth and negative velocity decreases depth.

The simulator prevents the profiler from going above the water surface:

```text
if depth < 0:
    depth = 0
    velocity = 0
```

This keeps the simulated vehicle from moving into negative depth.

## Sensor Model

The simulator includes a simple depth measurement model.

The measured depth is calculated by adding random noise to true depth:

```text
measured depth = true depth + noise
```

The noise is generated from a normal distribution.

This helps show how sensor noise can affect PID control and tolerance checking.

## Target Tolerance

The simulator checks whether the profiler is within the target tolerance:

```text
abs(measured depth - target depth) <= tolerance
```

If the profiler is inside the tolerance band during a hold state, the hold timer starts.

If it leaves the tolerance band, the hold timer resets.

This matches the behavior expected from a real profiler mission where the vehicle must actually stay near the target, not just touch it once.

## Graphs and Display

The simulator displays three main visual areas.

### Depth Plot

The depth plot shows:

* True depth
* Measured depth
* Target depth

The y-axis is inverted so deeper values appear lower on the graph.

### Actuator Plot

The actuator plot shows the actuator command in microseconds over time.

This helps show when the controller is commanding more buoyancy or less buoyancy.

### Depth Graphic

The depth graphic shows:

* A simplified water column
* Current profiler position
* Target depth
* Target tolerance band
* Current mission state
* Current velocity
* Current actuator command

This gives a quick visual reference for what the profiler is doing during the mission.

## Why This Is Useful for PID Tuning

A real profiler can be frustrating to tune because changes take time to show up. The vehicle has low available force, drag changes with velocity, and the actuator does not instantly cancel motion.

The simulator makes it easier to see how different PID gains affect:

* Rise time
* Overshoot
* Oscillation
* Settling behavior
* Hold performance
* Actuator saturation
* Mission completion

This makes it a useful first step before pool testing.

## Known Limitations

This simulator uses simplified physics.

Current limitations include:

* No full actuator delay model yet
* No detailed pressure sensor model
* No fluid turbulence model
* No wave or surface disturbance model
* No battery voltage effects
* No mechanical backlash model
* No real buoyancy engine geometry model
* No leak or trapped air behavior
* No data logging export yet
* No automatic PID tuning method yet

The simulator is still useful for controller development, but real testing is still required.

## Future Improvements

Possible future improvements include:

* Better actuator response model
* More detailed buoyancy engine simulation
* Config files loaded from JSON
* Data export to CSV
* More realistic pressure sensor simulation
* Adjustable drag model
* Additional controller options besides PID
* Feed-forward control
* Braking behavior before target depth
* Automatic PID tuning tools
* Web-based simulator view
* More profiler presets
* Mission editor

## Running the Simulator

Install the required Python packages:

```bash
pip install numpy matplotlib
```

Run the simulator:

```bash
python vp_sim.py
```

If the file has a different name, run:

```bash
python your_file_name.py
```

## Suggested Repo Structure

```text
vertical-profiler-sim/
  README.md
  vp_sim.py
  requirements.txt
  docs/
    images/
      velocity-proof-1.jpg
      velocity-proof-2.jpg
```

## Requirements

The simulator currently uses:

```text
numpy
matplotlib
```

Put this in `requirements.txt`:

```text
numpy
matplotlib
```

## Summary

The Vertical Profiler Simulator is a Python test bed for modeling vertical profiler behavior and tuning PID loops before testing on real hardware.

It models the main forces acting on the profiler, including weight, buoyancy, and drag. It also simulates depth, velocity, acceleration, actuator command, sensor noise, mission state changes, and PID control response.

The purpose is to make vertical profiler tuning easier, faster, and more understandable.
