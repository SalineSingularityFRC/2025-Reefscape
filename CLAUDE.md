# CLAUDE.md - FRC Team 5066 - 2025 Reefscape Robot

## Project Overview

This is the robot code for FRC Team 5066's 2025 competition robot for the Reefscape game. The robot is designed to intake and score both algae and coral game pieces, primarily scoring on the reef structure with the capability to place algae in the barge.

**Team Number:** 5066
**Competition Year:** 2025
**Game:** Reefscape
**Language:** Java 17
**Framework:** WPILib Command-Based Programming

## Tech Stack

### Core Libraries
- **WPILib 2025.3.2** - FRC robot framework
- **GradleRIO** - Build and deployment system
- **AdvantageKit** - Logging and replay framework for debugging

### Vendor Libraries
- **Phoenix 6 (25.2.2)** - CTRE motor controllers (Talon FX, etc.)
- **REVLib (2025.0.2)** - REV Robotics motor controllers (SparkMax, etc.)
- **PathPlannerLib (2025.2.3)** - Autonomous path planning and following
- **GrappleFRC (2025)** - Additional utilities
- **WPILib New Commands** - Command-based programming framework

## Architecture Overview

The codebase follows the **WPILib Command-Based Programming Pattern**. Key architectural principles:

- **Simpler is better than complex. Complex is better than complicated.**
- Subsystems encapsulate hardware and provide methods for commands to use
- Commands implement behaviors by calling subsystem methods
- RobotContainer wires everything together (subsystems, commands, triggers)

### Directory Structure

```
src/main/java/
├── frc/robot/
│   ├── Main.java                    - Entry point
│   ├── Robot.java                   - Robot lifecycle management
│   ├── RobotContainer.java          - Subsystem/command wiring
│   ├── Constants.java               - All robot constants (CAN IDs, speeds, etc.)
│   ├── PID.java                     - PID utilities
│   ├── AlertManager.java            - Alert/logging management
│   ├── commands/                    - Robot commands
│   │   ├── DriveController.java
│   │   ├── DriveToPose.java
│   │   ├── CameraDriveToPose.java
│   │   └── ...
│   ├── subsystems/                  - Robot subsystems
│   │   ├── SwerveSubsystem.java
│   │   ├── IntakeSubsystem.java
│   │   ├── AlgaeSubsystem.java
│   │   ├── AlgaeProcessorSubsystem.java
│   │   ├── TroughSubsystem.java
│   │   ├── ElevatorSubsystem.java
│   │   ├── ClimberSubsystem.java
│   │   └── LEDStatusSubsystem.java
│   └── SwerveClasses/               - Swerve drive implementation
│       ├── SwerveModule.java
│       ├── SwerveAngle.java
│       └── SwerveOdometry.java
└── lib/vision/                      - Vision processing
    ├── Limelight.java
    ├── LimelightHelpers.java
    └── RealSenseCamera.java

src/main/deploy/pathplanner/
├── autos/                           - Autonomous routines
└── paths/                           - Path definitions
```

## Key Subsystems

### Drive System
- **Type:** Swerve Drive
- **Implementation:** Custom swerve classes in `SwerveClasses/`
- **Key Files:**
  - `SwerveSubsystem.java` - Main drive subsystem
  - `SwerveModule.java` - Individual module control
  - `SwerveOdometry.java` - Position tracking

### Game Piece Handling
1. **IntakeSubsystem** - Intakes both algae and coral from the ground
2. **TroughSubsystem** - Transfers game pieces through the robot
3. **AlgaeSubsystem** - Handles algae scoring mechanisms
4. **AlgaeProcessorSubsystem** - Processes algae for scoring
5. **ElevatorSubsystem** - Vertical positioning for scoring

### Other Systems
- **ClimberSubsystem** - Endgame climbing mechanism
- **LEDStatusSubsystem** - Visual status indicators

## Vision Systems

### Limelight
- **Purpose:** Robot localization using AprilTags
- **Implementation:** `lib/vision/Limelight.java` and `LimelightHelpers.java`
- **Use Case:** Odometry corrections and pose estimation

### RealSense Camera
- **Purpose:** Detect and locate the center of reef poles for precise scoring
- **Implementation:** `lib/vision/RealSenseCamera.java`
- **Connection:** Runs on Raspberry Pi, communicates via NetworkTables
- **Use Case:** Vision-assisted scoring alignment

## Autonomous Strategy

The robot has autonomous routines that:
- **Score 3 Level 4 coral** from two different starting positions:
  - Top starting position
  - Bottom starting position
- Paths are defined in `src/main/deploy/pathplanner/`
- Uses PathPlannerLib for trajectory following

## Development Workflow

### Preferred Testing Approach
1. **Simulation First** - Test in WPILib simulation before deploying to hardware
   - Note: Simulation capabilities need enhancement (identified improvement area)
2. **AdvantageKit Logging** - Use for debugging and replay
3. **Iterative Testing** - Deploy to robot for real-world validation

### Build Commands
```bash
./gradlew build              # Build the project
./gradlew deploy             # Deploy to robot
./gradlew simulateJava       # Run robot simulation
./gradlew test               # Run tests
./gradlew replayWatch        # AdvantageKit replay watcher
```

### Common Tasks
- **Deploy to robot:** `./gradlew deploy` (robot must be connected)
- **Run simulation:** `./gradlew simulateJava`
- **View logs:** AdvantageKit logs are saved and can be replayed

## Code Patterns and Philosophy

### Command-Based Pattern
- Each subsystem extends `SubsystemBase`
- Commands extend `Command` or use inline commands
- Default commands run when no other commands are scheduled
- Commands are bound to triggers in `RobotContainer`

### Constants Organization
All constants are in `Constants.java`, organized by subsystem:
- `CanId.*` - CAN bus device IDs
- Subsystem-specific nested classes for motors, sensors, speeds, etc.

### Philosophy
> "Simpler is better than complex. Complex is better than complicated."

- Prefer straightforward solutions over clever ones
- Keep subsystems focused and commands composable
- Avoid over-engineering; add complexity only when needed
- Use WPILib patterns and best practices

## Important Files

### Configuration
- `build.gradle` - Gradle build configuration, dependencies, deploy settings
- `.wpilib/wpilib_preferences.json` - Team number and project settings
- `vendordeps/*.json` - Third-party library configurations

### Constants and Config
- `src/main/java/frc/robot/Constants.java` - All robot constants
- `src/main/deploy/pathplanner/` - PathPlanner autonomous paths and autos

### Core Robot Code
- `RobotContainer.java` - Command/subsystem binding and initialization
- `Robot.java` - Main robot class with mode handlers

### NetworkTables
- `networktables.json` - NetworkTables persistent values
- Used for communication with RealSense camera on Raspberry Pi

## Debugging and Logging

### AdvantageKit
- Primary logging framework
- Logs everything for post-match replay and analysis
- Use `Logger` class throughout code for telemetry
- Replay tool: `./gradlew replayWatch`

### Dashboard
- Uses Shuffleboard/Glass for live telemetry
- Configuration in `simgui.json` and `simgui-ds.json`

## Hardware Notes

### CAN IDs
All CAN device IDs are in `Constants.java` under `CanId` nested classes:
- Swerve modules
- Intake motors (left: 30, right: 31)
- Elevator, climber, algae mechanisms
- Sensors (laser/photo sensors)

### Sensors
- Laser/photo sensors for game piece detection
- Limelight for vision processing
- RealSense depth camera (external, via NetworkTables)

## Scoring Capabilities

- **Primary:** Score coral on reef at Level 4 (autonomous and teleop)
- **Secondary:** Place algae in barge
- Robot can handle both algae and coral game pieces
- Vision-assisted alignment for precise scoring

## Known Considerations

- Simulation framework needs enhancement for more sophisticated testing
- Follow WPILib command patterns consistently
- NetworkTables used for external vision processing (RealSense on Raspberry Pi)
- AdvantageKit provides comprehensive logging for debugging

## Getting Started (For Claude)

When working on this codebase:
1. Review `Constants.java` for device IDs and configuration
2. Check `RobotContainer.java` to understand command bindings
3. Look at subsystem files to understand robot capabilities
4. Test in simulation when possible before suggesting robot deployment
5. Maintain the command-based pattern in any new code
6. Keep code simple and maintainable per team philosophy
7. Use AdvantageKit logging for any new features
