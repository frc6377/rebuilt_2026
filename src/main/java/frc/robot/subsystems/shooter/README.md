# Hooded Shooter Subsystem

## Overview

This implementation features a dual-sided IO layer with independent Kraken X60 motors for redundancy and limp-mode operation. The hood angle is controlled by a Kraken x44 motor. Includes tunable PID gains and distance-based auto-aiming.

## Architecture

### Components

1. **Dual Flywheel System**
   - Left Kraken X60 (CAN ID 20)
   - Right Kraken X60 (CAN ID 21)
   - Independent velocity control (no follower/master configuration)
   - Anti-fighting logic via independent setpoints

2. **Adjustable Hood**
   - Kraken x44 (CAN ID 22)
   - Position-controlled adjustable pitch
   - Range: 0° to 45°

### Design Principles

#### Independent Motor Control
The flywheels are controlled independently to enable:
- **Limp-mode operation**: If one motor fails, the other continues functioning
- **No fighting**: Each motor receives its own velocity command
- **Failure detection**: Monitors velocity difference between motors

#### Redundancy Strategy
- Each flywheel has its own controller and feedback loop
- System continues operating if one side drops out
- Automatic failure detection based on velocity tracking

## Key Features

### 1. Independent Velocity Control
```java
shooter.setFlywheelVelocities(leftRPM, rightRPM);
```
Both flywheels can receive different commands, preventing motor "fighting" while maintaining throughput.

### 2. Limp-Mode Detection
The system automatically detects motor failures by monitoring:
- Velocity difference between left and right flywheels
- Inability to reach target velocity
- When a failure is detected, the failed motor is stopped and the system continues with the functioning motor

### 3. Hood Angle Control
```java
shooter.setHoodAngle(angleDegrees);
```
Adjustable hood for variable shot angles (0° to 45°).

### 4. Distance-Based Auto-Aiming
```java
shooter.prepareForDistance(distanceMeters);
```
Automatically calculates and sets optimal flywheel velocity and hood angle based on distance to target using interpolation maps.

### 5. Tunable PID Gains
All PID gains are tunable in real-time via NetworkTables:
- Flywheel: kP, kI, kD, kV, kS
- Hood: kP, kI, kD

## Usage

### Command Methods (in Shooter.java)

```java
// Spin up flywheels to 3000 RPM
shooter.spinUpFlywheels(3000.0);

// Set hood angle to 30 degrees
shooter.setHoodAngleCommand(30.0);

// Prepare for a shot (spin up + set angle)
shooter.prepareShot(3000.0, 30.0);

// Auto-aim for a specific distance
shooter.prepareForDistanceCommand(3.5); // 3.5 meters

// Prepare and wait until ready
shooter.prepareShotAndWait(3000.0, 30.0);

// Stop all motors
shooter.stopCommand();
```

### Button Bindings (in RobotContainer)

- **Right Trigger**: Spin up flywheels to 3000 RPM
- **Left Bumper**: Set hood to 20° (low shot)
- **Right Bumper**: Set hood to 35° (high shot)

## Configuration

All constants are defined in `ShooterConstants.java`:

### Motor IDs
```java
leftFlywheelMotorID = 20    // Kraken X60
rightFlywheelMotorID = 21   // Kraken X60
hoodMotorID = 22            // Kraken x44
```

### Units Library Constants
All physical constants use WPILib's units library:
```java
flywheelCurrentLimit = Amps.of(60.0)
hoodCurrentLimit = Amps.of(40.0)
maxFlywheelVelocity = RotationsPerSecond.of(100.0)  // 6000 RPM
minHoodAngle = Degrees.of(0.0)
maxHoodAngle = Degrees.of(45.0)
flywheelVelocityTolerance = RotationsPerSecond.of(100.0 / 60.0)  // 100 RPM
maxVelocityDifference = RotationsPerSecond.of(500.0 / 60.0)  // 500 RPM
```

### Tunable PID Gains
PID gains are tunable in real-time via NetworkTables under "Shooter/":
```java
// Flywheel velocity control (defaults)
Shooter/FlywheelKP = 0.1
Shooter/FlywheelKI = 0.0
Shooter/FlywheelKD = 0.0
Shooter/FlywheelKV = 0.12
Shooter/FlywheelKS = 0.0

// Hood position control (defaults)
Shooter/HoodKP = 10.0
Shooter/HoodKI = 0.0
Shooter/HoodKD = 0.5
```

### Distance-Based Shot Maps
Configure shot parameters for different distances in `Shooter.configureShotMaps()`:
```java
// Distance (meters) -> Velocity (RPM)
distanceToVelocityMap.put(1.0, 2000.0);
distanceToVelocityMap.put(3.0, 3000.0);
distanceToVelocityMap.put(5.0, 4000.0);

// Distance (meters) -> Hood Angle (degrees)
distanceToAngleMap.put(1.0, 15.0);
distanceToAngleMap.put(3.0, 25.0);
distanceToAngleMap.put(5.0, 35.0);
```

Values between defined points are automatically interpolated.

## AdvantageKit Integration

The shooter follows the AdvantageKit IO layer pattern:

- **ShooterIO**: Interface defining IO operations
- **ShooterIOKrakenX60**: Real hardware implementation
- **ShooterIOSim**: Physics simulation implementation
- **ShooterIOInputsAutoLogged**: Auto-generated logged inputs

All sensor data is automatically logged via AdvantageKit for replay and analysis.

## Safety Features

1. **Current Limiting**
   - Flywheels: 60A stator current limit
   - Hood: 40A stator current limit

2. **Position Limits**
   - Hood angle constrained to 0° - 45° range

3. **Failure Detection**
   - Automatic detection of motor failures
   - Limp-mode operation if one motor fails
   - Failed motors are automatically stopped

4. **Independent Control**
   - No follower configuration prevents cascading failures
   - Each motor has its own feedback loop

## Testing

### In Simulation
The shooter can be tested in simulation mode using the ShooterIOSim implementation, which provides:
- Physics-based flywheel simulation
- Single-jointed arm simulation for hood
- Realistic current draw and velocity tracking

### On Hardware
1. Verify CAN IDs match configuration
2. Check motor directions (left should be CCW, right should be CW)
3. Tune PID gains for your specific mechanism
4. Adjust gear ratios if needed

## Maintenance

### Resetting Failure Flags
If a motor has been flagged as failed but is now working:
```java
shooter.resetFailureFlags();
```

### Monitoring
Check AdvantageKit logs for:
- `Shooter/LeftFlywheelFailed`
- `Shooter/RightFlywheelFailed`
- `Shooter/InLimpMode`
- Velocity tracking and current draw

## Future Enhancements

- Auto-targeting based on vision
- Dynamic velocity/angle calculation
- Feed-forward models for faster spin-up
- Temperature-based current limiting
