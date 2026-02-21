# 2026Robot - Team 1405 FRC Robot Code

Competition robot code for the 2026 FRC season, built on the CTRE Phoenix 6 Swerve Drive framework.

## Table of Contents
- [Project Structure](#project-structure)
- [Getting Started](#getting-started)
- [Key Components](#key-components)
- [Adding Autonomous Commands](#adding-autonomous-commands)
- [Guides & Documentation](#guides--documentation)
- [Development Notes](#development-notes)

## Project Structure

```
2026Robot/
├── src/main/java/frc/robot/
│   ├── commands/           # Command implementations
│   │   ├── AutoPilot/     # AutoPilot-based autonomous commands
│   │   └── PidToPose/     # PID-based pose navigation commands
│   ├── subsystems/        # Robot subsystems
│   │   ├── CommandSwerveDrivetrain.java
│   │   ├── Climber.java
│   │   ├── Intake.java
│   │   ├── Hopper.java
│   │   ├── AdjustableHood.java
│   │   └── vision/        # Vision subsystem
│   ├── lib/               # Utilities and helper classes
│   │   ├── AutoCommands.java        # AUTO REGISTRATION - Start here!
│   │   ├── AprilTags.java          # AprilTag layout and overrides
│   │   ├── AllianceSymmetry.java   # Red/Blue alliance mirroring
│   │   ├── FinneyCommand.java      # Base command with logging
│   │   └── FinneyLogger.java       # Logging utility
│   ├── constants/         # Robot constants and configuration
│   ├── generated/         # CTRE-generated swerve code
│   ├── RobotContainer.java
│   └── Robot.java
├── Guides/                # Team guides and procedures
└── docs/                  # Technical documentation
```

## Getting Started

### Prerequisites
- WPILib 2026+ installed
- CTRE Phoenix 6 libraries (via vendordeps)
- Java 17+

### Building and Deploying
```bash
# Build the project
./gradlew build

# Deploy to robot
./gradlew deploy

# Run simulation
./gradlew simulateJava
```

## Key Components

### Swerve Drive System
This project uses the **CTRE Phoenix 6 Swerve Drive Generator** for drivetrain control:
- **Generated Code**: `src/main/java/frc/robot/generated/TunerConstants.java`, `CommandSwerveDrivetrain.java`
- **Drive Modes**: 
  - Field-centric (default)
  - Robot-centric
  - Snake Mode (intake follows movement direction)

📖 **See**: [How to Tune a Swerve Drive](docs/HowToTuneASwerveDrive.md)

### Vision System
Multi-camera AprilTag-based pose estimation for autonomous and assisted driving.

**Location**: `src/main/java/frc/robot/subsystems/vision/`

## Adding Autonomous Commands

### Quick Start - Adding a New Auto

1. **Create your command** in `commands/AutoPilot/AutoPilotCommands.java` or `commands/PidToPose/PidToPoseCommands.java`

2. **Register the command** in `lib/AutoCommands.java`:
   ```java
   public static void registerCommands(CommandSwerveDrivetrain drivetrain, Climber climber) {
       // Register your command with PathPlanner
       NamedCommands.registerCommand("MyAutoCommand", 
           new AutoPilotCommand.Builder(
               () -> new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90)),
               drivetrain,
               "MyAutoCommand"
           ).build()
       );
   }
   ```

3. **Add to chooser** in `configureAutos()` method:
   ```java
   commandsToAddToChooser.put("MyAuto", NamedCommands.getCommand("MyAutoCommand"));
   ```

### AutoPilot Commands
Uses the AutoPilot library for smooth, controlled autonomous navigation.

**Example**:
```java
new AutoPilotCommand.Builder(targetPoseSupplier, drivetrain, "CommandName")
    .withEntryAngle(Rotation2d.fromDegrees(90))     // Optional: approach angle
    .withFlipPoseForAlliance(true)                  // Optional: mirror for red alliance
    .withPointTowardsDuringMotion(() -> gamePiece)  // Optional: track target
    .withConstraints(customConstraints)              // Optional: custom motion profile
    .build();
```

**Key File**: `src/main/java/frc/robot/lib/AutoCommands.java`

📖 **AutoPilot Documentation**: https://therekrab.github.io/autopilot/

## Guides & Documentation

### Team Guides
Located in `Guides/`:
- [Competition Checklist](Guides/CompetitionChecklist.md) - Pre-match preparation
- [Download Match Logs for Replay](Guides/DownloadMatchLogsForReplay.md) - Post-match analysis
- [Subsystem Writing Guide](Guides/SubsystemWritingGuide.md) - How to create new subsystems
- [WPICal Calibration](Guides/WPICal_Calibration.md) - Camera calibration procedures
- [GitHub Workflow](Guides/Github/) - Git and version control practices

### Technical Documentation
Located in `docs/`:
- [How to Tune a Swerve Drive](docs/HowToTuneASwerveDrive.md)
- [Simulation Troubleshooting](docs/SimulationTroubleshooting.md)
- [Battery Status Monitoring](docs/BatteryStatus.md)
- [Offseason Projects](docs/OffseasonProjects.md)

## Development Notes

### Match Data & Analysis
- **Logs**: Stored in `logs/` directory (`.wpilog` format)
- **Analysis**: Use AdvantageScope for replay and telemetry
- **Layouts**: Pre-configured AdvantageScope and Elastic layouts in root directory

### Important Utilities

#### `AutoCommands.java`
Central registration point for all autonomous commands. **This is where you add new autos!**

#### `AprilTags.java`
Contains field AprilTag positions and field specific overrides.

#### `AllianceSymmetry.java`
Utilities for mirroring poses and paths between red and blue alliances.

#### `FinneyCommand.java` & `FinneyLogger.java`
Base command class with built-in logging and command tracking for better debugging.

## Contributing

1. Create a feature branch from `main`
2. Follow the [Subsystem Writing Guide](Guides/SubsystemWritingGuide.md)
3. Test in simulation

## Resources

- [WPILib Documentation](https://docs.wpilib.org/)
- [CTRE Phoenix 6 Docs](https://v6.docs.ctr-electronics.com/)
- [PathPlanner Documentation](https://pathplanner.dev/)
- [AutoPilot Library](https://therekrab.github.io/autopilot/)

---

**Team 1405** | FRC 2026 Season
