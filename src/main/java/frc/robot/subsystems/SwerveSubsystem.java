package frc.robot.subsystems;

import java.util.List;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.pose.GeneralPose;
import lib.pose.ScoreConfig.TargetState;
import lib.pose.ScoreConfig.NavigationTarget;
import lib.vision.Limelight;
import lib.vision.RealSenseCamera;
import frc.robot.SwerveClasses.SwerveModule;
import frc.robot.SwerveClasses.SwerveOdometry;
import frc.robot.commands.driving.CameraDriveToPose;
import frc.robot.commands.driving.DriveToPose2d;
import frc.robot.commands.driving.FollowPath;
import frc.robot.commands.driving.CameraDriveToPose.PoseAndTarget;
import org.littletonrobotics.junction.Logger;

/*
 * This class provides functions to drive at a given angle and direction,
 * and performs the calculations required to achieve that
 */
public class SwerveSubsystem extends SubsystemBase {

  /*
   * This class should own the pidgeon 2.0 IMU gyroscope that we will be using and
   * a dictionary that will house
   * all of our SwerveModules
   */
  private Pigeon2 gyro;

  private final int FL = 0;
  private final int FR = 1;
  private final int BL = 2;
  private final int BR = 3;

  private SwerveModule[] swerveModules = new SwerveModule[4];
  private SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
  private final Translation2d[] vectorKinematics = new Translation2d[4];

  private final SwerveDriveKinematics swerveDriveKinematics;
  private ChassisSpeeds chassisSpeeds;
  private double gyroZero = 0;

  private PIDController rotationController;
  // private SimpleMotorFeedforward feedforwardRotation;

  private double pastRobotAngle;
  private double currentRobotAngle;
  private double pastRobotAngleDerivative;
  private double currentRobotAngleDerivative;
  private boolean isRotating;

  private SwerveOdometry odometry;

  private Field2d field;

  private StructPublisher<Pose2d> publisher;

  private NetworkTableInstance inst;
  private NetworkTable table;

  private Boolean BlueAlliance;

  private DataLog log;
  private DoubleLogEntry flEncoderPositionLog;
  private DoubleLogEntry frEncoderPositionLog;
  private DoubleLogEntry blEncoderPositionLog;
  private DoubleLogEntry brEncoderPositionLog;
  private DoubleLogEntry flPositionLog;
  private DoubleLogEntry frPositionLog;
  private DoubleLogEntry blPositionLog;
  private DoubleLogEntry brPositionLog;
  private DoubleLogEntry pidgeonAccelerationXLog;
  private DoubleLogEntry pidgeonAccelerationYLog;
  private DoubleLogEntry pidgeonAccelerationZLog;
  private DoubleLogEntry pidgeonAngularVelocityXLog;
  private DoubleLogEntry pidgeonAngularVelocityYLog;
  private DoubleLogEntry pidgeonAngularVelocityZLog;
  private DoubleLogEntry pidgeonTimeLog;

  // Simulation variables
  private Pigeon2SimState gyroSim;
  private TalonFXSimState[] driveMotorSims = new TalonFXSimState[4];
  private CANcoderSimState[] cancoderSims = new CANcoderSimState[4];
  private double[] simDriveMotorPositions = new double[4]; // Track positions in rotations

  /*
   * This constructor should create an instance of the pidgeon class, and should
   * construct four copies of the
   * SwerveModule class and add them to our SwerveModule dictionary
   * Use values from the Constants.java class
   */
  public SwerveSubsystem(Limelight leftLL, Limelight rightLL) {
    log = DataLogManager.getLog();

    BlueAlliance = true;

    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("datatable");

    gyro = new Pigeon2(Constants.CanId.CanCoder.GYRO, Constants.Canbus.DRIVE_TRAIN);

    vectorKinematics[FL] = new Translation2d(Constants.Measurement.WHEEL_BASE / 2,
        Constants.Measurement.TRACK_WIDTH / 2);
    vectorKinematics[FR] = new Translation2d(Constants.Measurement.WHEEL_BASE / 2,
        -Constants.Measurement.TRACK_WIDTH / 2);
    vectorKinematics[BL] = new Translation2d(-Constants.Measurement.WHEEL_BASE / 2,
        Constants.Measurement.TRACK_WIDTH / 2);
    vectorKinematics[BR] = new Translation2d(-Constants.Measurement.WHEEL_BASE / 2,
        -Constants.Measurement.TRACK_WIDTH / 2);

    swerveDriveKinematics = new SwerveDriveKinematics(vectorKinematics);

    chassisSpeeds = new ChassisSpeeds();
    swerveModules[FL] = new SwerveModule(
        Constants.CanId.Swerve.Drive.FL,
        Constants.CanId.Swerve.Angle.FL,
        Constants.CanId.CanCoder.FL,
        Constants.SwerveModule.WheelOffset.FL,
        Constants.Canbus.DRIVE_TRAIN,
        Constants.Inverted.FL,
        Constants.Inverted.ANGLE,
        "FL");
    swerveModules[FR] = new SwerveModule(
        Constants.CanId.Swerve.Drive.FR,
        Constants.CanId.Swerve.Angle.FR,
        Constants.CanId.CanCoder.FR,
        Constants.SwerveModule.WheelOffset.FR,
        Constants.Canbus.DRIVE_TRAIN,
        Constants.Inverted.FR,
        Constants.Inverted.ANGLE,
        "FR");
    swerveModules[BL] = new SwerveModule(
        Constants.CanId.Swerve.Drive.BL,
        Constants.CanId.Swerve.Angle.BL,
        Constants.CanId.CanCoder.BL,
        Constants.SwerveModule.WheelOffset.BL,
        Constants.Canbus.DRIVE_TRAIN,
        Constants.Inverted.BL,
        Constants.Inverted.ANGLE,
        "BL");
    swerveModules[BR] = new SwerveModule(
        Constants.CanId.Swerve.Drive.BR,
        Constants.CanId.Swerve.Angle.BR,
        Constants.CanId.CanCoder.BR,
        Constants.SwerveModule.WheelOffset.BR,
        Constants.Canbus.DRIVE_TRAIN,
        Constants.Inverted.BR,
        Constants.Inverted.ANGLE,
        "BR");

    odometry = new SwerveOdometry(this, swerveDriveKinematics, leftLL, rightLL);
    odometry.resetPosition();

    Supplier<ChassisSpeeds> supplier_chasis = () -> {
      ChassisSpeeds temp = getChassisSpeed();
      return temp;
    };

    Consumer<ChassisSpeeds> consumer_chasis = ch_speed -> {
      // Store the chassis speeds for simulation
      this.chassisSpeeds = ch_speed;

      // Convert to module states and set them
      SwerveModuleState[] modules = swerveDriveKinematics.toSwerveModuleStates(ch_speed);
      setModuleStates(modules);

      // Log that PathPlanner is commanding movement
      Logger.recordOutput("PathPlanner/CommandedVelocity/vx", ch_speed.vxMetersPerSecond);
      Logger.recordOutput("PathPlanner/CommandedVelocity/vy", ch_speed.vyMetersPerSecond);
      Logger.recordOutput("PathPlanner/CommandedVelocity/omega", ch_speed.omegaRadiansPerSecond);
    };
    Supplier<Pose2d> supplier_position = () -> {
      return odometry.getEstimatedPosition();
    };
    Consumer<Pose2d> consumer_position = pose -> {
      odometry.setPosition(pose);
    };

    SwerveModuleState[] modules = swerveDriveKinematics.toSwerveModuleStates(getChassisSpeed());
    setModuleStates(modules);

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // config = Constants.PathplannerConfig.ChassisRobotConfig;

    AutoBuilder.configure(
        supplier_position, // Robot pose supplier
        consumer_position, // Method to reset odometry (will be called if your auto has a starting pose)
        supplier_chasis, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        consumer_chasis, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(Constants.PidGains.PathPlanner.translation.P, Constants.PidGains.PathPlanner.translation.I,
                Constants.PidGains.PathPlanner.translation.D), // Translation PID constants
            new PIDConstants(Constants.PidGains.PathPlanner.rotation.P, Constants.PidGains.PathPlanner.rotation.I,
                Constants.PidGains.PathPlanner.rotation.D) // Rotation PID constants
        ),
        config,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

    publisher = table.getStructTopic("Final Odometry Position", Pose2d.struct).publish();
    rotationController = new PIDController(Constants.Drive.ROTATION_CORRECTION_KP.getValue(),
        Constants.Drive.ROTATION_CORRECTION_KI.getValue(),
        Constants.Drive.ROTATION_CORRECTION_KD.getValue());
    rotationController.setTolerance(0.5);
    // feedforwardRotation = new SimpleMotorFeedforward(0.05, 0);

    pastRobotAngle = 0;
    currentRobotAngle = 0;
    pastRobotAngleDerivative = 0;
    currentRobotAngleDerivative = 0;
    isRotating = false;

    flEncoderPositionLog = new DoubleLogEntry(log, "FL encoder position");
    frEncoderPositionLog = new DoubleLogEntry(log, "FR encoder position");
    blEncoderPositionLog = new DoubleLogEntry(log, "BL encoder position");
    brEncoderPositionLog = new DoubleLogEntry(log, "BR encoder position");
    flPositionLog = new DoubleLogEntry(log, "FL position");
    frPositionLog = new DoubleLogEntry(log, "FR position");
    blPositionLog = new DoubleLogEntry(log, "BL position");
    brPositionLog = new DoubleLogEntry(log, "BR position");
    pidgeonAccelerationXLog = new DoubleLogEntry(log, "Pidgeon Acceleration X");
    pidgeonAccelerationYLog = new DoubleLogEntry(log, "Pidgeon Acceleration Y");
    pidgeonAccelerationZLog = new DoubleLogEntry(log, "Pidgeon Acceleration Z");
    pidgeonAngularVelocityXLog = new DoubleLogEntry(log, "Pidgeon Angular Velocity X");
    pidgeonAngularVelocityYLog = new DoubleLogEntry(log, "Pidgeon Angular Velocity Y");
    pidgeonAngularVelocityZLog = new DoubleLogEntry(log, "Pidgeon Angular Velocity Z");
    pidgeonTimeLog = new DoubleLogEntry(log, "Pidgeon Time");

    // Initialize Field2d for AdvantageScope visualization
    field = new Field2d();
    SmartDashboard.putData("Field", field);

    // Initialize simulation objects
    if (Constants.Modes.currentMode == Constants.Mode.SIM) {
      gyroSim = gyro.getSimState();
      gyroSim.setSupplyVoltage(12.0);

      for (int i = 0; i < 4; i++) {
        driveMotorSims[i] = swerveModules[i].getDriveMotor().getSimState();
        driveMotorSims[i].setSupplyVoltage(12.0);

        cancoderSims[i] = swerveModules[i].getCANcoder().getSimState();
        cancoderSims[i].setSupplyVoltage(12.0);

        // Initialize simulation position tracking
        simDriveMotorPositions[i] = 0.0;
      }
    }

    initializeDriveCommands();

  }

  /*
   * In the future, to reduce latency, refactor all the drive commands to not be
   * deferred.
   */
  public void initializeDriveCommands() {
    // Empty for now - will be populated with drive command initialization in the future
  }

  public Supplier<Pose2d> supplier_position = () -> {
    return odometry.getEstimatedPosition();
  };

  /**
   * Method for driving via controller
   * @param rotation
   * @param x
   * @param y
   * @param fieldCentric
   * @param mulitplier
   */
  public void drive(
      double rotation,
      double x,
      double y,
      boolean fieldCentric,
      double mulitplier) {

    double currentRobotAngle = gyro.getYaw().getValueAsDouble();
    double currentRobotAngleRadians = getRobotAngle();

    // this is to make sure if both the joysticks are at neutral position, the robot
    // and wheels
    // don't move or turn at all
    // 0.08 value can be increased if the joystick is increasingly inaccurate at
    // neutral position
    if (Math.abs(x) < 0.06
        && Math.abs(y) < 0.06
        && Math.abs(rotation) < 0.08) {

      this.stop();
      return;
    }

    x = x * mulitplier;
    y = y * mulitplier;
    rotation = rotation * mulitplier;

    double robotX = x;
    double robotY = y;
    if (fieldCentric) {
      double difference = -(currentRobotAngleRadians % (2 * Math.PI));
      robotX = -y * Math.sin(difference)
          + x * Math.cos(difference);
      robotY = y * Math.cos(difference)
          + x * Math.sin(difference);
    }

    // The following is for heading correction
    currentRobotAngleDerivative = currentRobotAngle - pastRobotAngle;
    if (currentRobotAngleDerivative == 0) {
      currentRobotAngleDerivative = pastRobotAngleDerivative;
    }

    if ((pastRobotAngleDerivative > 0 && currentRobotAngleDerivative < 0) ||
        (pastRobotAngleDerivative < 0 && currentRobotAngleDerivative > 0)) {
      isRotating = false;
    }

    // For correcting angular position when not rotating manually
    double gyroYaw = gyro.getYaw().getValueAsDouble();
    if (Math.abs(rotation) > 0.08 * mulitplier || isRotating) {
      isRotating = true;
      rotationController.setSetpoint(gyroYaw);
      rotationController.reset();
    } else {
      rotation = rotationController.calculate(gyroYaw);
    }

    // SmartDashboard.putNumber("Rotation Correction/Setpoint: Robot Angle",
    // rotationController.getSetpoint());
    // SmartDashboard.putNumber("Rotation Correction/Plant state: Robot Angle",
    // gyro.getYaw().getValueAsDouble());
    // SmartDashboard.putNumber("Control Effort: Calculated Rotation Speed",
    // rotation);
    // SmartDashboard.putBoolean("Is bot turning", isRotating);

    this.chassisSpeeds.vxMetersPerSecond = robotX;
    this.chassisSpeeds.vyMetersPerSecond = robotY;
    this.chassisSpeeds.omegaRadiansPerSecond = rotation;

    SwerveModuleState[] modules = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(modules);

    pastRobotAngle = currentRobotAngle;
    pastRobotAngleDerivative = currentRobotAngleDerivative;
  }

  public void updateRotationPIDSetpoint() {
    rotationController.setSetpoint(gyro.getYaw().getValueAsDouble());
    rotationController.reset();
  }

  public ChassisSpeeds getChassisSpeed() {
    return swerveDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void logData() {

    // Logging total speed
    ChassisSpeeds speeds = getChassisSpeed();
    double vx = speeds.vxMetersPerSecond;
    double vy = speeds.vyMetersPerSecond;
    double totalSpeed = Math.sqrt(vx * vx + vy * vy);
    SmartDashboard.putNumber("SwerveData/Chassis Speed", totalSpeed);
    // To log data into these entries, wherever you would have used SmartDashboard,
    // use:
    // flEncoderPositionLog.append(swerveModules[FL].getEncoderPosition());
    // frEncoderPositionLog.append(swerveModules[FR].getEncoderPosition());
    // blEncoderPositionLog.append(swerveModules[BL].getEncoderPosition());
    // brEncoderPositionLog.append(swerveModules[BR].getEncoderPosition());

    // flPositionLog.append(swerveModules[FL].getPosition());
    // frPositionLog.append(swerveModules[FR].getPosition());
    // blPositionLog.append(swerveModules[BL].getPosition());
    // brPositionLog.append(swerveModules[BR].getPosition());

    // pidgeonAccelerationXLog.append(gyro.getAccelerationX().getValueAsDouble());
    // pidgeonAccelerationYLog.append(gyro.getAccelerationY().getValueAsDouble());
    // pidgeonAccelerationZLog.append(gyro.getAccelerationZ().getValueAsDouble());

    // pidgeonAngularVelocityXLog.append(gyro.getAngularVelocityXDevice().getValueAsDouble());
    // pidgeonAngularVelocityYLog.append(gyro.getAngularVelocityYDevice().getValueAsDouble());
    // pidgeonAngularVelocityZLog.append(gyro.getAngularVelocityZDevice().getValueAsDouble());

    // pidgeonTimeLog.append(gyro.getUpTime().getValueAsDouble());
  }

  public void periodic() {

    var Alliance = DriverStation.getAlliance();
    if (Alliance.isPresent()) {
      if (Alliance.get() == DriverStation.Alliance.Red) {
        BlueAlliance = false;
      }
    }

    SmartDashboard.putBoolean("Swerve/Is Blue", BlueAlliance);

    publisher.set(odometry.getEstimatedPosition());

    // Update Field2d widget
    Pose2d currentPose = odometry.getEstimatedPosition();
    field.setRobotPose(currentPose);

    // Log robot pose for AdvantageScope
    Logger.recordOutput("Odometry/Robot", currentPose);

    // Log swerve module states for visualization
    SwerveModuleState[] moduleStates = getModuleStates();
    Logger.recordOutput("Swerve/ModuleStates", moduleStates);

    // Log desired vs actual states for debugging
    Logger.recordOutput("Swerve/DesiredStates",
        swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds));

    // Log chassis speeds for debugging
    Logger.recordOutput("Swerve/ChassisSpeedsCommanded/vx", chassisSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Swerve/ChassisSpeedsCommanded/vy", chassisSpeeds.vyMetersPerSecond);
    Logger.recordOutput("Swerve/ChassisSpeedsCommanded/omega", chassisSpeeds.omegaRadiansPerSecond);

    // Log gyro angle
    Logger.recordOutput("Odometry/GyroAngle", gyro.getYaw().getValueAsDouble());

    // logData();

  }

  public void disabledPeriodic() {
  }

  @Override
  public void simulationPeriodic() {
    // Log chassis speeds being used in simulation
    Logger.recordOutput("Simulation/ChassisSpeedsInSim/vx", chassisSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Simulation/ChassisSpeedsInSim/vy", chassisSpeeds.vyMetersPerSecond);
    Logger.recordOutput("Simulation/ChassisSpeedsInSim/omega", chassisSpeeds.omegaRadiansPerSecond);

    // Update simulated gyro based on commanded angular velocity
    if (gyroSim != null) {
      // Get current commanded angular velocity in degrees per second
      double angularVelocityDegPerSec = Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond);

      // Update gyro yaw (integrate angular velocity)
      // Multiply by 0.02 (20ms loop period)
      gyroSim.addYaw(angularVelocityDegPerSec * 0.02);

      // Log simulation data
      Logger.recordOutput("Simulation/AngularVelocity", angularVelocityDegPerSec);
    }

    // Update simulated motor positions and angles based on commanded states
    SwerveModuleState[] desiredStates = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Log the desired states being calculated in simulation
    Logger.recordOutput("Simulation/DesiredStatesInSim", desiredStates);

    for (int i = 0; i < 4; i++) {
      if (driveMotorSims[i] != null && cancoderSims[i] != null) {
        // Simulate drive motor (velocity and position)
        // Convert meters per second to rotations per second
        // Then multiply by gear ratio to get motor rotations per second
        double wheelRotationsPerSec = desiredStates[i].speedMetersPerSecond /
            (2 * Math.PI * Constants.Measurement.WHEELRADIUS);
        double motorRotationsPerSec = wheelRotationsPerSec * Constants.SwerveModule.GearRatio.DRIVE;

        // Update position (integrate velocity over 20ms loop time)
        simDriveMotorPositions[i] += motorRotationsPerSec * 0.02;

        // Set the motor simulation state
        // Use setRawRotorPosition for absolute position
        driveMotorSims[i].setRawRotorPosition(simDriveMotorPositions[i]);
        driveMotorSims[i].setRotorVelocity(motorRotationsPerSec);

        // Simulate CANcoder (angle encoder)
        // Set the absolute position to match the desired angle
        // Convert from radians to rotations (CANcoder reports in rotations)
        double desiredAngleRotations = desiredStates[i].angle.getRadians() / (2 * Math.PI);
        cancoderSims[i].setRawPosition(desiredAngleRotations);
        cancoderSims[i].setVelocity(0); // Assume angle changes instantly in simulation

        // Log motor positions for debugging
        if (i == 0) { // Log just the front-left for debugging
          Logger.recordOutput("Simulation/FL_MotorPositionTracked", simDriveMotorPositions[i]);
          Logger.recordOutput("Simulation/FL_MotorPosition", swerveModules[i].getDriveMotor().getPosition().getValueAsDouble());
          Logger.recordOutput("Simulation/FL_MotorVelocity", swerveModules[i].getDriveMotor().getVelocity().getValueAsDouble());
          Logger.recordOutput("Simulation/FL_DesiredSpeed", desiredStates[i].speedMetersPerSecond);
          Logger.recordOutput("Simulation/FL_DesiredMotorRotationsPerSec", motorRotationsPerSec);
          Logger.recordOutput("Simulation/FL_DesiredAngle", desiredStates[i].angle.getDegrees());
          Logger.recordOutput("Simulation/FL_CANcoderPosition", swerveModules[i].getCANcoder().getAbsolutePosition().getValueAsDouble());
        }
      }
    }
  }

  public void updateOdometry() {
    odometry.update();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    swerveModules[FL].setDesiredState(desiredStates[0]);
    swerveModules[FR].setDesiredState(desiredStates[1]);
    swerveModules[BL].setDesiredState(desiredStates[2]);
    swerveModules[BR].setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[FL] = swerveModules[FL].getState();
    states[FR] = swerveModules[FR].getState();
    states[BL] = swerveModules[BL].getState();
    states[BR] = swerveModules[BR].getState();
    return states;
  }

  /**
   * This function returns the angle (in radians) of the robot based on the value
   * from the pidgeon 2.0
   */
  public double getRobotAngle() {
    return gyro.getRotation2d().plus(Rotation2d.fromDegrees(180.0)).getRadians() - gyroZero;
  }

  /**
   * Accounts for where foward is for swerve pos estimation (red/blue alliance)
   */
  public Rotation2d getRobotRotation2dForOdometry() {
    if (BlueAlliance) {
      return new Rotation2d(getRobotAngle());
    } else {
      return new Rotation2d(getRobotAngle() + Math.PI);
    }
  }

  /**
   * Returns algular speed of robot
   */
  public double getAngularChassisSpeed() {
    return gyro.getAngularVelocityZWorld().getValueAsDouble();
  }

  public Command resetGyroCommand() {
    return runOnce(
        () -> {
          resetGyro();
        });
  }

  public Command updateRotationPIDSetpointCommand() {
    return runOnce(
        () -> {
          updateRotationPIDSetpoint();
        });
  }

  public Command xMode() {
    return runOnce(
        () -> {
          SwerveModuleState[] modules = new SwerveModuleState[4];
          modules[FL] = new SwerveModuleState(0.02, new Rotation2d(Math.PI / 4.0));
          modules[FR] = new SwerveModuleState(0.02, new Rotation2d((-5.0 * Math.PI) / 4.0));
          modules[BR] = new SwerveModuleState(0.02, new Rotation2d((Math.PI) / 4.0));
          modules[BL] = new SwerveModuleState(0.02, new Rotation2d((-5.0 * Math.PI) / 4.0));
          setModuleStates(modules);
        });
  }

  private void stop() {
    swerveModules[FR].stopDriving();
    swerveModules[FL].stopDriving();
    swerveModules[BR].stopDriving();
    swerveModules[BL].stopDriving();
  }

  /**
   * Stops all swerve modules from moving
   */
  public Command stopDriving() {
    return runOnce(
        () -> {
          this.stop();
        });
  }

  public Command endDrive() {
    return Commands.sequence(
      stopDriving(),
      updateRotationPIDSetpointCommand()
      );
  }

  /**
   * Resets gryo for field centric driving
   */
  public void resetGyro() {
    gyroZero = gyro.getRotation2d().plus(Rotation2d.fromDegrees(180.0)).getRadians();
  }

  public SwerveModule getSwerveModule(int module) {
    return swerveModules[module];
  }

  public Command setBrakeModeCommand() {
    return runOnce(
        () -> {
          setBrakeMode();
        });
  }

  public Command setCoastModeCommand() {
    return runOnce(
        () -> {
          setCoastMode();
        });
  }

  public void setBrakeMode() {
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setBrakeMode();
    }
  }

  public void setCoastMode() {
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setCoastMode();
    }
  }

  public boolean isCoast() {
    return swerveModules[0].isCoast();
  }

  /**
   * Filters to closest pose and the state corresponding to that pose
   */
  public GeneralPose filterClosestState(TargetState targetState) {

    List<GeneralPose> filteredPoses;
    Pose2d closestPose;
    GeneralPose targetGeneralPose;

    // Filter to wanted poses to compare
    filteredPoses = Constants.Poses.generalPoses.stream()
        .filter(
            (p) -> p.getSide() == targetState.getSide() &&
                p.getNavTarget() == targetState.getNavTarget())
        .toList();

    // Closest Pose2d to current position based on filteredPoses
    closestPose = getClosestPose2d(filteredPoses);

    if (closestPose == null) {
      return null;
    }

    // Finds corresponding general pose (NOTE: Will need to change to be more
    // efficient by keeping track of index)
    targetGeneralPose = filteredPoses.stream()
        .filter(
            (p) -> p.getPose2d().equals(closestPose))
        .findFirst()
        .get();

    // If already known elevator setpoint, then set the targetState to include
    // setpoint
    if (targetState.getSetpoint() != null) {
      targetGeneralPose = targetGeneralPose.withTargetState(targetState);
    }

    // Flip Pose2d if on red alliance
    targetGeneralPose = targetGeneralPose.withPose2d(getFieldAdjustedPose2d(targetGeneralPose.getPose2d()));

    return targetGeneralPose;
  }

  /**
   * Returns the closest Pose2d in the list from current robot pose
   */
  public Pose2d getClosestPose2d(List<GeneralPose> filteredPoses) {
    List<Pose2d> poses = filteredPoses.stream().map((rp) -> {
      return rp.getPose2d();
    }).toList();

    Pose2d ourPose = odometry.getEstimatedPosition();

    Pose2d fieldAdjustedPose = getFieldAdjustedPose2d(ourPose);
    Pose2d nearest = fieldAdjustedPose.nearest(poses);

    if (nearest == null) {
      return null;
    }

    return nearest;
  }

  /**
   * Returns position on field (flipped if red alliance)
   */
  public Pose2d getFieldAdjustedPose2d(Pose2d pose) {
    return BlueAlliance ? pose : FlippingUtil.flipFieldPose(pose);
  }

  /**
   * Drives to whatever Pose2d the supplier returns.
   * 
   * @param poseSupplier supplies the target pose at command initialization & each
   *                     execution
   */
  public Command driveToPose2d(Supplier<Pose2d> poseSupplier, NavigationTarget targetObject) {
    // Directly return your path-following command,
    // letting DriveToPose itself handle following the trajectory.
    return new DriveToPose2d(this, supplier_position, poseSupplier, targetObject);
  }

  /**
   * Convenience overload for deferring command creation to runtime
   */
  public Command driveToGeneralPose(GeneralPose generalPose) {

    // Follow a pathplanner path based on chosen ending point if intaking algae
    if (generalPose.getNavTarget() == NavigationTarget.ALGAE) {
      return executePathPlannerPath(generalPose);
    }
    // Wrap the static pose in a supplier.
    return driveToPose2d(() -> generalPose.getPose2d(), generalPose.getNavTarget());
  }

  /**
   * Returns the chosen pathplanner path to execute
   * TODO: Make this non deferred since this is implicetly deferring the command.
   */
  public Command executePathPlannerPath(GeneralPose generalPose) {
    return new FollowPath(() -> generalPose, this);
  }

  /**
   * Backing away from the reef after intaking algae
   */
  public Command backAwayFromReef(GeneralPose generalPose) {
    return new FollowPath(() -> generalPose.withTargetState(TargetState.ALGAE_BACK_AWAY), this);
  }

  /*
   * Returns if we are on the blue alliance
   */
  public boolean isBlueAlliance() {
    return BlueAlliance;
  }

  /**
   * Drives to pose supplied by camera
   */
  public Command cameraDriveToPose(RealSenseCamera cam, Pose2d pose) {
    return new DeferredCommand(() -> {
      Pose2d closestTargetPose = pose;

      // return new CameraDriveToPose(this, supplier_position, () -> {
      // Pose2d targetPose = cam.getReefPose().get();
      // Pose2d robotPose = supplier_position.get();
      // if (targetPose == null) {
      // return closestTargetPose;
      // }

      // Pose2d newRobotPose = robotPose.plus(new Transform2d(targetPose.getX(),
      // targetPose.getY(), new Rotation2d(0)));
      // return new Pose2d(newRobotPose.getX(), newRobotPose.getY(),
      // closestTargetPose.getRotation());
      // });

      return new CameraDriveToPose(this, () -> {
        Pose2d targetPose = cam.getFinalReefPose().get();
        Pose2d robotPose = supplier_position.get();

        if (targetPose == null) {
          // If we don't see a camera reading, use the robot position and the auto reef
          // target
          SmartDashboard.putString("realsensecamera/pidmode", "normal");
          return new PoseAndTarget(robotPose, closestTargetPose);
        }

        // We make the robot 0,0,currentHeading
        // We make the target to be the reef pole position according to the camera, but
        // use the target heading of our auto reef position
        SmartDashboard.putString("realsensecamera/pidmode", "camera");
        return new PoseAndTarget(
            new Pose2d(0, 0, robotPose.getRotation()),
            new Pose2d(targetPose.getX(), targetPose.getY(), closestTargetPose.getRotation()));

        // Pose2d newRobotPose = robotPose.plus(new Transform2d(targetPose.getX(),
        // targetPose.getY(), new Rotation2d(0)));
        // return new Pose2d(newRobotPose.getX(), newRobotPose.getY(),
        // closestTargetPose.getRotation());
      });

    }, Set.of(this));
  }

  /**
   * Drives to barge shoot point. (RobotX, RobotY) -> (BargePoseX, RobotY)
   */
  public Command driveToBarge() {
    return new DeferredCommand(() -> {
      if (BlueAlliance) {
        return driveToPose2d(
            () -> new Pose2d(Constants.Poses.bargeXBlue, supplier_position.get().getTranslation().getY(),
                new Rotation2d(0)),
            NavigationTarget.BARGE);
      } else {
        // Flipping bargeXBlue to red side (for 2025 field only)
        return driveToPose2d(() -> new Pose2d(Units.feetToMeters(57.573) - Constants.Poses.bargeXBlue,
            supplier_position.get().getTranslation().getY(),
            new Rotation2d(Math.PI)), NavigationTarget.BARGE);
      }
    }, Set.of(this));
  }

  /**
   * Drives close to barge shoot point. (RobotX, RobotY) -> (BargePoseX, RobotY)
   */
  public Command driveCloseToBarge() {
    return new DeferredCommand(() -> {
      if (BlueAlliance) {
        return driveToPose2d(
            () -> new Pose2d(Constants.Poses.bargeXFarBlue, supplier_position.get().getTranslation().getY(),
                new Rotation2d(0)),
            NavigationTarget.CLOSE_TO_BARGE);
      } else {
        // Flipping bargeXFarBlue to red side (for 2025 field only)
        return driveToPose2d(() -> new Pose2d(Units.feetToMeters(57.573) - Constants.Poses.bargeXFarBlue,
            supplier_position.get().getTranslation().getY(), new Rotation2d(Math.PI)),
            NavigationTarget.CLOSE_TO_BARGE);
      }
    }, Set.of(this));
  }
}
