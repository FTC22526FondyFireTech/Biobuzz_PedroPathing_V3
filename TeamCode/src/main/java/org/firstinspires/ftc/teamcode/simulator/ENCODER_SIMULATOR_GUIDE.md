# Hardware-Independent Encoder Simulator

## Overview

This simulator lets you test autonomous paths **without any robot hardware**. It:

1. **Simulates motor physics** (acceleration, velocity, encoder generation)
2. **Calculates odometry** from the four mecanum drive wheels (translation) plus a simulated gyro (heading) - no dead-wheel odometry pods required
3. **Integrates with FTC Panels** for real-time visualization
4. **Produces the same encoder values** Pedro Pathing would receive from hardware

This is perfect for:
- Testing autonomous paths before hardware is ready
- Debugging odometry calculations
- Tuning motor physics constants
- Integration testing without a robot

---

## Architecture

```
Your OpMode (simRun())
    ↓ setMotorPowers(fl, fr, bl, br)      ↓ rotate command
4x MotorSimulator                     GyroSimulator
    - Takes power input                   - Takes rotate power input
    - Simulates acceleration curve         - Simulates acceleration curve
    - Generates encoder ticks              - Generates heading (deg)
    ↓                                      ↓
         OdometrySimulator
    - Recovers forward/strafe from all 4 wheel ticks via
      mecanum forward kinematics (inverse of the drive mixing)
    - Takes heading directly from GyroSimulator (immune to
      wheel-slip drift, like a real IMU)
    - Returns x, y, heading
    ↓
FTC Panels Telemetry
    - Display simulation on dashboard
```

---

## Installation

### Step 1: Add Simulator Classes to Your Project

Copy these Java files to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/simulator/`:

1. `MotorSimulator.java` - Single motor physics
2. `OdometrySimulator.java` - Three-wheel odometry calculator
3. `SimulatedOpMode.java` - Base class for simulated OpModes

### Step 2: Create Your Simulated Autonomous OpMode

Create a new OpMode that extends `SimulatedOpMode`:

```java
@Autonomous(name = "Simulated Auto", group = "Simulator")
public class MySimulatedAuto extends SimulatedOpMode {
    
    @Override
    public void initialize() {
        initSimulation(); // This sets up all simulators
    }
    
    @Override
    protected void simRun() {
        // Set motor powers and test your autonomous logic
        setMotorPowers(0.5, 0.5, 0.5, 0.5); // Drive forward
    }
}
```

### Step 3: Add Widget to FTC Panels

Copy `encoder-simulator-widget.jsx` to your FTC Panels dashboard and add it:

```jsx
import SimulatorVisualizationWidget from './components/encoder-simulator-widget';

// In your dashboard:
<SimulatorVisualizationWidget />
```

---

## Usage Examples

### Example 1: Simple Drive Forward

```java
@Override
protected void simRun() {
    // Drive forward at 50% power for 3 seconds
    if (getRuntime() < 3.0) {
        setMotorPowers(0.5, 0.5, 0.5, 0.5);
    } else {
        setMotorPowers(0, 0, 0, 0);
        requestOpModeStop();
    }
    
    // Check position on FTC Panels
    telemetry.addData("X", getSimulatedPose().getX());
    telemetry.addData("Y", getSimulatedPose().getY());
    telemetry.update();
}
```

### Example 2: Strafe Right

```java
@Override
protected void simRun() {
    // Mecanum strafe: opposite corners forward
    setMotorPowers(-0.5, 0.5, 0.5, -0.5); // FL=reverse, FR=forward, BL=forward, BR=reverse
}
```

### Example 3: Rotate Clockwise

```java
@Override
protected void simRun() {
    // Rotate: left side forward, right side backward
    setMotorPowers(0.5, -0.5, 0.5, -0.5);
}
```

### Example 4: Using Teleop Simulation

```java
@Override
protected void simRun() {
    // Simulate gamepad input
    double forward = 0.5;  // from gamepad.left_stick_y
    double strafe = 0.3;   // from gamepad.left_stick_x
    double turn = 0.2;     // from gamepad.right_stick_x
    
    simulateTeleopDrive(forward, strafe, turn);
}
```

### Example 5: Sequential Actions

```java
ElapsedTime actionTimer = new ElapsedTime();

@Override
public void initialize() {
    initSimulation();
    actionTimer.reset();
}

@Override
protected void simRun() {
    double time = actionTimer.seconds();
    
    if (time < 2.0) {
        setMotorPowers(0.5, 0.5, 0.5, 0.5); // Move forward
    } else if (time < 4.0) {
        setMotorPowers(-0.5, 0.5, 0.5, -0.5); // Strafe right
    } else if (time < 6.0) {
        setMotorPowers(0.5, -0.5, 0.5, -0.5); // Rotate
    } else {
        setMotorPowers(0, 0, 0, 0);
        requestOpModeStop();
    }
}
```

---

## Physics Tuning

The simulator includes these physics constants in `MotorSimulator.java`:

```java
private static final double MAX_RPM = 435;              // Max motor speed
private static final double ACCELERATION_TIME = 0.150;  // Time to reach max
private static final double MOTOR_INERTIA = 0.002;      // kg·m²

private static final double ENCODER_TICKS_PER_REV = 288;    // Core Hex 1.1
private static final double WHEEL_DIAMETER_INCHES = 3.78;   // goBILDA mecanum
```

### Tuning for Your Specific Motors/Wheels

1. **MAX_RPM**: Match your actual motor (e.g., REV Core Hex = 435 RPM)
2. **ACCELERATION_TIME**: How quickly motor reaches target speed
   - Lower = snappier response
   - Higher = more realistic mass/inertia
3. **ENCODER_TICKS_PER_REV**: Your motor's encoder resolution
4. **WHEEL_DIAMETER_INCHES**: Your actual wheel size

---

## Telemetry Keys

The simulator publishes to FTC Panels under these keys:

**Robot Pose:**
- `sim/x (in)` - X position in inches
- `sim/y (in)` - Y position in inches
- `sim/heading (deg)` - Robot heading in degrees

**Motor State (for each motor: fl, fr, bl, br):**
- `sim/motor/{motor}/power` - Power command [-1, 1]
- `sim/motor/{motor}/rpm` - Current velocity
- `sim/motor/{motor}/ticks` - Total encoder ticks

**Three-Wheel Encoders:**
- `sim/encoder/leftFront` - Left forward encoder
- `sim/encoder/rightRear` - Right rear encoder
- `sim/encoder/rightFront` - Right strafe encoder

**Odometry Debug:**
- `sim/delta/forward (in)` - Forward motion this frame
- `sim/delta/strafe (in)` - Strafe motion this frame
- `sim/delta/heading (rad)` - Heading change this frame

---

## Integration with Pedro Pathing

The `OdometrySimulator` uses the exact same three-wheel localization math as Pedro Pathing's `ThreeWheelConstants`. This means:

- Simulated encoder values are identical to hardware in terms of pose calculation
- You can validate your localization constants without a robot
- Odometry deltas show exactly what Pedro Pathing is calculating

For full Pedro Pathing integration, you would:

1. Create a mock `ThreeWheelLocalizer` that reads from `OdometrySimulator`
2. Inject simulated encoders into Pedro Pathing's `Follower`
3. Run full autonomous path following in simulation

(This is an advanced integration - let me know if you need help with this step!)

---

## Debugging

### Pose not updating?
- Check that `simRun()` is calling `setMotorPowers()` with non-zero values
- Verify FTC Panels is connected and showing `sim/*` telemetry keys
- Check the driver station console for errors

### Motors accelerating too fast/slow?
- Adjust `ACCELERATION_TIME` in `MotorSimulator`
- Lower value = faster acceleration
- Higher value = more realistic (accounts for robot mass)

### Encoder values growing too fast/slow?
- Check `ENCODER_TICKS_PER_REV` matches your motor
- Check `WHEEL_DIAMETER_INCHES` is correct
- Verify MAX_RPM matches your actual motor

### Odometry pose incorrect?
- Translation (x/y) comes from mecanum forward kinematics over all 4 wheels -
  verify `drive()`'s mixing (`FL=f+r+w, FR=f-r-w, BL=f-r+w, BR=f+r-w`) matches
  what `OdometrySimulator.update()` inverts
- Heading comes from `GyroSimulator`, not wheel ticks - if heading is right but
  x/y is wrong, the bug is in the wheel kinematics, not the gyro

### Heading rotating wrong direction?
- Check `GyroSimulator.MAX_ANGULAR_VELOCITY_DEG_PER_SEC` sign/magnitude
- Verify the rotate power passed to `gyro.update()` matches the same `rotate`
  argument passed into `drive()` (same sign, same source)

---

## Testing Checklist

Before deploying to hardware:

- [ ] Create `SimulatedAutoTest` OpMode
- [ ] Copy MotorSimulator, OdometrySimulator, SimulatedOpMode to project
- [ ] Build project successfully (no compilation errors)
- [ ] Deploy to robot/emulator
- [ ] Open FTC Panels dashboard
- [ ] Add encoder-simulator-widget to dashboard
- [ ] Run SimulatedAutoTest
- [ ] Watch robot position update on widget
- [ ] Verify motor powers, RPM, encoder ticks populate
- [ ] Verify three-wheel encoder values increasing
- [ ] Check odometry deltas match expected motion

---

## Next Steps

### Easy:
- Test different motor power combinations
- Verify acceleration curves match expectations
- Tune ACCELERATION_TIME for realistic feel

### Medium:
- Create autonomous paths using `setMotorPowers()` sequences
- Add state machines for multi-phase autonomous
- Use `getSimulatedPose()` to validate final positions

### Advanced:
- Integrate with Pedro Pathing's `Follower` for path following sim
- Create mock hardware providers that feed simulated values
- Build full end-to-end autonomous testing without hardware

---

## Troubleshooting Tips

**Simulator creates motors but no movement?**
- Verify `OpModeIsActive()` check passes
- Check that `simRun()` is actually being called
- Look for exceptions in Driver Station logcat

**Motor RPM shows 0 but power is non-zero?**
- Check MotorSimulator.update() is being called
- Verify dt (delta time) is reasonable (not 0)
- Check that ACCELERATION_TIME makes sense for your update rate

**Encoders accumulating but pose not changing?**
- OdometrySimulator.update() might not be called
- Check that all three encoder values are being read
- Verify three-wheel offsets are correct (not zero)

---

## Files Included

```
simulator/
├── MotorSimulator.java           - Single motor physics
├── OdometrySimulator.java        - Three-wheel odometry
├── SimulatedOpMode.java          - Base class
├── SimulatedAutoTest.java        - Example OpMode
└── encoder-simulator-widget.jsx  - FTC Panels widget
```

Start with `SimulatedAutoTest.java` as a template and modify `simRun()` for your autonomous logic.
