package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.vision.Limelight;
import lib.vision.RealSenseCamera;
import frc.robot.SwerveClasses.SwerveModule;
import frc.robot.SwerveClasses.SwerveOdometry;
import frc.robot.commands.CameraDriveToPose;
import frc.robot.commands.CameraDriveToPose.PoseAndTarget;
import frc.robot.commands.DriveToBargePose;
import frc.robot.commands.DriveToPose;

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
      SwerveModuleState[] modules = swerveDriveKinematics.toSwerveModuleStates(ch_speed);
      setModuleStates(modules);
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

  }

  public Supplier<Pose2d> supplier_position = () -> {
    return odometry.getEstimatedPosition();
  };

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

  public void initialize() {
    // gyro.reset();
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

    // SmartDashboard.putBoolean("Heading Issue/Is Blue", BlueAlliance);

    publisher.set(odometry.getEstimatedPosition());

    // logData();

  }

  public void disabledPeriodic() {
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

  /*
   * This function returns the angle (in radians) of the robot based on the value
   * from the pidgeon 2.0
   */
  public double getRobotAngle() {
    return gyro.getRotation2d().plus(Rotation2d.fromDegrees(180.0)).getRadians() - gyroZero;
  }

  // Accounts for where foward is for swerve pos estimation (red/blue alliance)
  public Rotation2d getRobotRotation2dForOdometry() {
    if (BlueAlliance) {
      return new Rotation2d(getRobotAngle());
    } else {
      return new Rotation2d(getRobotAngle() + Math.PI);
    }
  }

  /*
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

  public Command stopDriving() {
    return runOnce(
        () -> {
          this.stop();
        });
  }

  /*
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

  /*
   * Gets closest reef pose based on which side specified (left or right)
   */
  public Pose2d getClosestReef(AutoScoreTarget target) {

    List<ReefPose> posesForSide;

    // if (BlueAlliance) {
    // posesForSide = reefPoses.stream().filter((p) -> p.side ==
    // target.side).toList();
    // } else {
    posesForSide = reefPosesBlue.stream().filter((p) -> p.side != target.side).toList();
    // }
    // return posesForSide.get(0).pose;

    List<Pose2d> poses = posesForSide.stream().map((rp) -> {
      return rp.pose;
    }).toList();

    Pose2d ourPose = odometry.getEstimatedPosition();

    Pose2d fieldAdjustedPose = BlueAlliance ? ourPose : FlippingUtil.flipFieldPose(ourPose);
    Pose2d nearest = fieldAdjustedPose.nearest(poses);
    if (nearest == null) {
      return ourPose;
    }

    if (BlueAlliance) {
      return nearest;
      // return AutoBuilder.pathfindToPose(targetPose, constraints, 0);
    } else {
      return FlippingUtil.flipFieldPose(nearest);
      // return AutoBuilder.pathfindToPoseFlipped(targetPose, constraints, 0);
    }

    // SmartDashboard.putString("AutoScore/Chosen Reef", ((WrappedPose2d)
    // nearest).reefPose.name);
    // SmartDashboard.putNumber("AutoScore/Chosen Reef/X", nearest.getX());
    // SmartDashboard.putNumber("AutoScore/Chosen Reef/Y", nearest.getY());
  }

  /*
   * Gets closest source pose based on which side specified (left or right)
   */
  public Pose2d getClosestSource(AutoScoreTarget target) {

    List<ReefPose> posesForSide;

    // if (BlueAlliance) {
    // posesForSide = sourcePoses.stream().filter((p) -> p.side ==
    // target.side).toList();
    // } else {
    posesForSide = sourcePosesBlue.stream().filter((p) -> p.side != target.side).toList();
    // }

    List<Pose2d> poses = posesForSide.stream().map((rp) -> {
      // WrappedPose2d np = new WrappedPose2d(rp.pose.getX(), rp.pose.getY(),
      // rp.pose.getRotation());
      // np.sourcePose = rp;
      // return (Pose2d) np;
      return rp.pose;
    }).toList();

    Pose2d ourPose = odometry.getEstimatedPosition();

    Pose2d fieldAdjustedPose = BlueAlliance ? ourPose : FlippingUtil.flipFieldPose(ourPose);
    Pose2d nearest = fieldAdjustedPose.nearest(poses);
    if (nearest == null) {
      return ourPose;
    }

    return nearest;
  }

  record ReefPose(String name, ReefFacetSide side, Pose2d pose) {
  };

  // Blue alliance only since we flip if red alliance (from pathplanner)
  static List<ReefPose> reefPosesBlue = List.of(
      new ReefPose("A", ReefFacetSide.LEFT, new Pose2d(3.20, 4.193, new Rotation2d(Math.toRadians(0)))),
      new ReefPose("B", ReefFacetSide.RIGHT, new Pose2d(3.20, 3.863, new Rotation2d(Math.toRadians(0)))),
      new ReefPose("C", ReefFacetSide.LEFT, new Pose2d(3.701, 2.999, new Rotation2d(Math.toRadians(60.0)))),
      new ReefPose("D", ReefFacetSide.RIGHT, new Pose2d(3.992, 2.835, new Rotation2d(Math.toRadians(60.0)))),
      new ReefPose("E", ReefFacetSide.LEFT, new Pose2d(4.984, 2.827, new Rotation2d(Math.toRadians(120.0)))),
      new ReefPose("F", ReefFacetSide.RIGHT, new Pose2d(5.275, 2.992, new Rotation2d(Math.toRadians(120.0)))),
      new ReefPose("G", ReefFacetSide.LEFT, new Pose2d(5.750, 3.863, new Rotation2d(Math.toRadians(180.0)))),
      new ReefPose("H", ReefFacetSide.RIGHT, new Pose2d(5.750, 4.19, new Rotation2d(Math.toRadians(180.0)))),
      new ReefPose("I", ReefFacetSide.LEFT, new Pose2d(5.246, 5.014, new Rotation2d(Math.toRadians(240.0)))),
      new ReefPose("J", ReefFacetSide.RIGHT, new Pose2d(4.962, 5.170, new Rotation2d(Math.toRadians(240.0)))),
      new ReefPose("K", ReefFacetSide.LEFT, new Pose2d(4.014, 5.163, new Rotation2d(Math.toRadians(300.0)))),
      new ReefPose("L", ReefFacetSide.RIGHT, new Pose2d(3.731, 5.014, new Rotation2d(Math.toRadians(300.0)))));

  // Blue alliance only since we flip if red alliance (from pathplanner)
  static List<ReefPose> sourcePosesBlue = List.of(
      new ReefPose("Left Source", ReefFacetSide.LEFT, new Pose2d(1.395, 7.387, new Rotation2d(Math.toRadians(306.0)))),
      new ReefPose("Right Source", ReefFacetSide.RIGHT,
          new Pose2d(1.480, 0.750, new Rotation2d(Math.toRadians(54.0)))));

  // Blue alliance only since we flip if red alliance
  static double bargeXBlue = 7.95;
  static double bargeXFarBlue = bargeXBlue - 0.5;

  /*
   * Drives to closest reef pose based on which side specified (left or right)
   */
  public Command drivetoReefPose(AutoScoreTarget target) {
    return new DeferredCommand(() -> {
      Pose2d targetPose = getClosestReef(target);
      // PathConstraints constraints = new PathConstraints(1.5, 1.5, .5, .5);

      // if (BlueAlliance) {
      // return new DriveToPose(this, supplier_position, () -> targetPose);
      // // return AutoBuilder.pathfindToPose(targetPose, constraints, 0);
      // } else {
      // return new DriveToPose(this, supplier_position, () ->
      // FlippingUtil.flipFieldPose(targetPose));
      // // return AutoBuilder.pathfindToPoseFlipped(targetPose, constraints, 0);
      // }

      return new DriveToPose(this, supplier_position, () -> targetPose);

    }, Set.of(this));
  }

  /*
   * Drives to closest coral source pose based on which side specified (left or
   * right)
   */
  public Command drivetoSourcePose(AutoScoreTarget target) {
    return new DeferredCommand(() -> {
      Pose2d targetPose = getClosestSource(target);
      // PathConstraints constraints = new PathConstraints(1.5, 1.5, .5, .5);

      if (BlueAlliance) {
        return new DriveToPose(this, supplier_position, () -> targetPose);
        // return AutoBuilder.pathfindToPose(targetPose, constraints, 0);
      } else {
        return new DriveToPose(this, supplier_position, () -> FlippingUtil.flipFieldPose(targetPose));
        // return AutoBuilder.pathfindToPoseFlipped(targetPose, constraints, 0);
      }

    }, Set.of(this));
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
  public Command cameraDriveToPose(RealSenseCamera cam, AutoScoreTarget closestTarget) {
    return new DeferredCommand(() -> {
      Pose2d closestTargetPose = getClosestReef(closestTarget);

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

  /*
   * Drives to barge shoot point. (RobotX, RobotY) -> (BargePoseX, RobotY)
   */
  public Command driveToBargePose() {
    return new DeferredCommand(() -> {

      if (BlueAlliance) {
        return new DriveToBargePose(this, supplier_position,
            () -> new Pose2d(bargeXBlue, supplier_position.get().getTranslation().getY(), new Rotation2d(0)));
      } else {
        // Flipping bargeXBlue to red side (for 2025 field only)
        return new DriveToBargePose(this, supplier_position,
            () -> new Pose2d(Units.feetToMeters(57.573) - bargeXBlue, supplier_position.get().getTranslation().getY(),
                new Rotation2d(Math.PI)));
      }

    }, Set.of(this));
  }

  /*
   * Drives close to barge shoot point. (RobotX, RobotY) -> (BargePoseX, RobotY)
   */
  public Command driveCloseToBargePose() {
    return new DeferredCommand(() -> {

      if (BlueAlliance) {
        return new DriveToPose(this, supplier_position,
            () -> new Pose2d(bargeXFarBlue, supplier_position.get().getTranslation().getY(), new Rotation2d(0)));
      } else {
        // Flipping bargeXFarBlue to red side (for 2025 field only)
        return new DriveToPose(this, supplier_position,
            () -> new Pose2d(Units.feetToMeters(57.573) - bargeXFarBlue,
                supplier_position.get().getTranslation().getY(), new Rotation2d(Math.PI)));
      }

    }, Set.of(this));
  }

  enum ReefFacetSide {
    LEFT, RIGHT
  }

  public static enum AutoScoreTarget {
    L4_LEFT(ReefFacetSide.LEFT),
    L4_RIGHT(ReefFacetSide.RIGHT),
    L3_LEFT(ReefFacetSide.LEFT),
    L3_RIGHT(ReefFacetSide.RIGHT),
    L2_LEFT(ReefFacetSide.LEFT),
    L2_RIGHT(ReefFacetSide.RIGHT),
    L1_LEFT(ReefFacetSide.LEFT),
    L1_RIGHT(ReefFacetSide.RIGHT);

    public ReefFacetSide side;

    AutoScoreTarget(ReefFacetSide side) {
      this.side = side;
    }
  }
}
