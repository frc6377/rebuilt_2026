# HowdyBot 2026 — FRC Team 6377

[![CI](https://github.com/frc6377/rebuild_2026/actions/workflows/main.yml/badge.svg)](https://github.com/frc6377/rebuild_2026/actions/workflows/main.yml)

Robot code for **FRC Team 6377 HowdyBot** competing in the **2026 FIRST Robotics Competition (FUEL)** season.
Written in Java using WPILib's Command-Based framework, with AdvantageKit for structured logging and CTRE Phoenix 6 for motor control.

---

## Game: FUEL 2026

The robot scores **FUEL** (game pieces) by launching them into the **Hub**. Key mechanisms:

- **Intake** — extends to collect FUEL from the floor
- **Indexer** — stages FUEL from the intake into the shooter path
- **Upgoer (Feeder)** — pushes FUEL up into the flywheels when a shot is ready
- **Shooter** — dual Kraken X60 flywheels (left + right) spin up to ~6 000 RPM to launch FUEL
- **Hood** — adjustable exit ramp (25°–80°) controls shot arc
- **Drivetrain** — swerve drive using four Kraken X60 modules, Pigeon 2 IMU

---

## Software Architecture

This project follows a strict **layered IO architecture** (AdvantageKit pattern):

```
SubsystemBase  (periodic loop, command factories, state tracking)
   └── IO Interface  (hardware abstraction)
         ├── IOKrakenX60  (real hardware — CTRE Phoenix 6)
         └── IOSim        (physics simulation — maple-sim / WPILib)
```

Split subsystems (e.g., left/right shooter) live in separate sub-packages:

```
subsystems/
  shooter/
    Shooter.java                  ← Container: instantiates left + right, no logic
    ShooterConstants.java         ← Shared constants
    left/
      LeftShooter.java            ← SubsystemBase, command factories
      LeftShooterIO.java          ← Interface + @AutoLog inputs
      LeftShooterIOKrakenX60.java ← Real hardware
      LeftShooterIOSim.java       ← Simulation
      LeftShooterConstants.java   ← Per-side constants (CAN IDs, PID gains)
    right/
      (mirror of left/)
  intake/
    roller/   ← roller IO layer
    extender/ ← extender IO layer
  indexer/
  upgoer/
  hood/
  drive/      ← swerve drive with odometry thread
  vision/     ← PhotonVision + Limelight AprilTag pose estimation
  superstructure/ ← Coordinates shooter, hood, upgoer, and game-piece state
  signaling/  ← LED patterns
```

---

## Key Libraries

| Library | Version | Purpose |
|---|---|---|
| [WPILib](https://docs.wpilib.org) | 2026.2.1 | Core framework, units, commands |
| [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit) | — | Deterministic logging (`@AutoLog`, `Logger`, `@AutoLogOutput`) |
| [CTRE Phoenix 6](https://api.ctr-electronics.com/phoenix6/release/java/) | 26.1.0 | TalonFX (Kraken X60) motor control, status signals |
| [PathPlanner](https://pathplanner.dev) | 2026.1.2 | Autonomous path following |
| [maple-sim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim) | — | Full physics drivetrain + intake simulation |
| [PhotonVision](https://photonvision.org) | — | AprilTag vision pose estimation |

---

## Subsystem Summary

| Subsystem | Hardware | Notes |
|---|---|---|
| **Drive** | 4× Kraken X60, Pigeon 2 | Field-relative swerve, odometry, SysId routines |
| **Left Shooter** | 1 Kraken X60 lead + 1 follower | Velocity-controlled flywheel, SysId support |
| **Right Shooter** | 1 Kraken X60 | Velocity-controlled flywheel (currently disabled) |
| **Hood** | 1 Kraken X60 | Position-controlled exit ramp angle |
| **Upgoer** | 1 Kraken X60 | Feeds FUEL into flywheels on shot command |
| **Indexer** | 1 motor | Stages FUEL from intake to upgoer |
| **Intake Roller** | 1 motor | Collects FUEL from floor |
| **Intake Extender** | 1 motor + CANCoder | Extends/retracts intake arm |
| **Vision** | 2 cameras (Limelight/PhotonVision) | AprilTag pose correction |
| **Superstructure** | — | State machine coordinating shooter, hood, upgoer |

---

## Operator Interface

The robot uses an **Xbox controller** (or keyboard in simulation):

| Button | Action |
|---|---|
| Hold spin-up | Spin up shooter at current manual setpoint |
| Hold fire | Feed FUEL through upgoer + indexer |
| D-Pad / bumpers | Select flywheel speed preset (2100–3300 RPM) |
| Intake trigger | Extend + run roller + run indexer |
| Outtake trigger | Reverse roller + reverse indexer |
| Back | Stop all superstructure |
| Start | Zero drivebase heading |
| Misc | Unjam, zero intake extender |

---

## Autonomous

Autonomous routines are built with **PathPlanner** and selected via a dashboard chooser.

Named commands registered for PathPlanner:

| Command | Action |
|---|---|
| `Spin Up Shooter` | Spin flywheels to 3 600 RPM |
| `Spin Up Shooter and Wait` | Spin up and wait until at speed |
| `Shoot` | Fire for up to 5 s |
| `Extend Intake` | Deploy intake arm |
| `Intake` | Run intake + indexer for up to 6 s |
| `Index` | Run indexer for up to 3 s |

SysId routines are also available in the auto chooser for characterization.

---

## Running the Robot

### Prerequisites

- [WPILib 2026](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) installed
- Java 21

### Common Gradle Tasks

```bash
# Build (no deploy)
./gradlew build

# Apply code formatting (Spotless)
./gradlew spotlessApply

# Check formatting (CI)
./gradlew spotlessCheck

# Deploy to roboRIO
./gradlew deploy

# Run simulation
./gradlew simulateJava

# Download all dependencies
./gradlew downloadAll

# Clean build artifacts
./gradlew clean

# Build + format in one step
./gradlew spotlessApply build
```

---

## Project Conventions

- **4-space indentation**, no tabs; opening braces on the same line.
- All physical quantities use **WPILib typed units** (`RPM.of(...)`, `Amps.of(...)`, etc.) — never bare `double`.
- CAN IDs and feature flags live in `Constants.java` — never hardcoded inline.
- Hardware is gated by `Constants.EnabledSubsystems.*` flags.
- Logging keys follow `"SubsystemName/FieldName"` format (PascalCase field, no spaces).
- See [`.github/copilot-instructions.md`](.github/copilot-instructions.md) for full coding style guide.

---

## References

- [What Is "Command-Based" Programming?](https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html)
- [Structuring a Command-Based Robot Project](https://docs.wpilib.org/en/stable/docs/software/commandbased/structuring-command-based-project.html)
- [Organizing Command-Based Robot Projects](https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html)
- [GradleRIO Tasks](https://docs.wpilib.org/en/stable/docs/software/advanced-gradlerio/gradlew-tasks.html)
- [Spotless Formatting](https://docs.wpilib.org/en/stable/docs/software/advanced-gradlerio/code-formatting.html)
