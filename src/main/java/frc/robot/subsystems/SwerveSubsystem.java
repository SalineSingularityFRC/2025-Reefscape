package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.SwerveClasses.SwerveModule;
import frc.robot.SwerveClasses.SwerveOdometry;


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
  private SimpleMotorFeedforward feedforwardRotation;

  private double pastRobotAngle;
  private double currentRobotAngle;
  private double pastRobotAngleDerivative;
  private double currentRobotAngleDerivative;
  private boolean isRotating;

  private SwerveOdometry odometry;

  private StructPublisher<Pose2d> publisher;

  private NetworkTableInstance inst;
  private NetworkTable table;
  /*
   * This constructor should create an instance of the pidgeon class, and should
   * construct four copies of the
   * SwerveModule class and add them to our SwerveModule dictionary
   * Use values from the Constants.java class
   */
  public SwerveSubsystem() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("datatable");

    gyro = new Pigeon2(Constants.CanId.CanCoder.GYRO, Constants.Canbus.DRIVE_TRAIN);

    vectorKinematics[FL] = new Translation2d(Constants.Measurement.WHEEL_BASE / 2, Constants.Measurement.TRACK_WIDTH / 2);
    vectorKinematics[FR] = new Translation2d(Constants.Measurement.WHEEL_BASE / 2, -Constants.Measurement.TRACK_WIDTH / 2);
    vectorKinematics[BL] = new Translation2d(-Constants.Measurement.WHEEL_BASE / 2, Constants.Measurement.TRACK_WIDTH / 2);
    vectorKinematics[BR] = new Translation2d(-Constants.Measurement.WHEEL_BASE / 2, -Constants.Measurement.TRACK_WIDTH / 2);

    swerveDriveKinematics = new SwerveDriveKinematics(vectorKinematics);

    chassisSpeeds = new ChassisSpeeds();
    swerveModules[FL] = new SwerveModule(
        Constants.CanId.Swerve.Drive.FL,
        Constants.CanId.Swerve.Angle.FL,
        Constants.CanId.CanCoder.FL,
        Constants.WheelOffset.FL,
        Constants.Canbus.DRIVE_TRAIN,
        Constants.Inverted.FL,
        Constants.Inverted.ANGLE,
        "FL");
    swerveModules[FR] = new SwerveModule(
        Constants.CanId.Swerve.Drive.FR,
        Constants.CanId.Swerve.Angle.FR,
        Constants.CanId.CanCoder.FR,
        Constants.WheelOffset.FR,
        Constants.Canbus.DRIVE_TRAIN,
        Constants.Inverted.FR,
        Constants.Inverted.ANGLE,
        "FR");
    swerveModules[BL] = new SwerveModule(
        Constants.CanId.Swerve.Drive.BL,
        Constants.CanId.Swerve.Angle.BL,
        Constants.CanId.CanCoder.BL,
        Constants.WheelOffset.BL,
        Constants.Canbus.DRIVE_TRAIN,
        Constants.Inverted.BL,
        Constants.Inverted.ANGLE,
        "BL");
    swerveModules[BR] = new SwerveModule(
        Constants.CanId.Swerve.Drive.BR,
        Constants.CanId.Swerve.Angle.BR,
        Constants.CanId.CanCoder.BR,
        Constants.WheelOffset.BR,
        Constants.Canbus.DRIVE_TRAIN,
        Constants.Inverted.BR,
        Constants.Inverted.ANGLE,
        "BR");

    odometry = new SwerveOdometry(this, vectorKinematics);
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
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

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
          // Boolean supplier that controls when the path will be mirrored for the red alliance
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
    rotationController = new PIDController(Constants.PidGains.rotationCorrection.rotation.P, 
      Constants.PidGains.rotationCorrection.rotation.I, 
      Constants.PidGains.rotationCorrection.rotation.D);
    rotationController.setTolerance(0.5);
    feedforwardRotation = new SimpleMotorFeedforward(0.05, 0);

    pastRobotAngle = 0;
    currentRobotAngle = 0;
    pastRobotAngleDerivative = 0;
    currentRobotAngleDerivative = 0;
    isRotating = false;
  }

  public void drive(
          double rotation,
          double x,
          double y,
          boolean fieldCentric) { 
    
    double currentRobotAngle = gyro.getYaw().getValueAsDouble();
    double currentRobotAngleRadians = getRobotAngle();

    SmartDashboard.putNumber("Angular Z Speed", gyro.getAngularVelocityZWorld().getValueAsDouble());
    SmartDashboard.putNumber("current robot angle", gyro.getYaw().getValueAsDouble());

    // this is to make sure if both the joysticks are at neutral position, the robot
    // and wheels
    // don't move or turn at all
    // 0.05 value can be increased if the joystick is increasingly inaccurate at
    // neutral position
    if (Math.abs(x) < 0.07
        && Math.abs(y) < 0.07
        && Math.abs(rotation) < 0.05) {

      this.stop();
      return;
    }

    double robotX = x; 
    double robotY = y;
    if (fieldCentric) {
      double difference = -(currentRobotAngleRadians % (2 * Math.PI));
      robotX = -y * Math.sin(difference)
          + x * Math.cos(difference);
      robotY = y * Math.cos(difference)
          + x * Math.sin(difference);
    }

    currentRobotAngleDerivative = currentRobotAngle - pastRobotAngle;
    if(currentRobotAngleDerivative == 0) {
      currentRobotAngleDerivative = pastRobotAngleDerivative;
    }

    SmartDashboard.putNumber("Angle Derivative", currentRobotAngleDerivative);

    if((pastRobotAngleDerivative > 0 && currentRobotAngleDerivative < 0) ||
       (pastRobotAngleDerivative < 0 && currentRobotAngleDerivative > 0)) {
      isRotating = false;
    }

    // For correcting angular position when not rotating manually
    if (Math.abs(rotation) > 0.05 || isRotating) {
      isRotating = true;
      rotationController.setSetpoint(gyro.getYaw().getValueAsDouble());
      rotationController.reset();
      // rotationController.calculate(gyro.getYaw().getValueAsDouble());
      // feedforwardRotation.calculate(rotation);
    }
    else {
      rotation = rotationController.calculate(gyro.getYaw().getValueAsDouble());
      // rotation += feedforwardRotation.calculate(rotation);
    }

    SmartDashboard.putNumber("Setpoint: Robot Angle", rotationController.getSetpoint());
    SmartDashboard.putNumber("Plant state: Robot Angle", gyro.getYaw().getValueAsDouble());
    SmartDashboard.putNumber("Control Effort: Calculated Rotation Speed", rotation);
    SmartDashboard.putBoolean("Is bot turning", isRotating);

    this.chassisSpeeds = new ChassisSpeeds(robotX, robotY, rotation);

    SwerveModuleState[] modules = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(modules);
    
    pastRobotAngle = currentRobotAngle;
    pastRobotAngleDerivative = currentRobotAngleDerivative;
  }

  public void updateRotationPIDSetpoint() {
    rotationController.setSetpoint(gyro.getYaw().getValueAsDouble());
  }

  public ChassisSpeeds getChassisSpeed() {
    return swerveDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void periodic() {
    odometry.update();
    publisher.set(odometry.getEstimatedPosition());
  }

  public void disabledPeriodic() {
  }

  public void updateOdometry() {
      odometry.update();
  }

  /*
   * Odometry
   */
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
    return (((gyro.getYaw().getValueAsDouble() - gyroZero)) * Math.PI)
        / 180; // returns in counterclockwise hence why 360 minus
    // it is gyro.getAngle() - 180 because the pigeon for this robot is facing
    // backwards
  }

  public double getAngularChassisSpeed() {
    return gyro.getAngularVelocityZWorld().getValueAsDouble();
  }

  public Command resetGyroCommand() {
    return runOnce(
        () -> {
          resetGyro();
        });
  }

  public Command rotate90() {
    return runOnce(
        () -> {

          ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

          SwerveModuleState[] modules = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
          modules[FL].angle = new Rotation2d(swerveModules[FL].getAngleClamped() + Math.PI / 8);
          modules[FR].angle = new Rotation2d(swerveModules[FR].getAngleClamped() + Math.PI / 8);
          modules[BR].angle = new Rotation2d(swerveModules[BR].getAngleClamped() + Math.PI / 8);
          modules[BL].angle = new Rotation2d(swerveModules[BL].getAngleClamped() + Math.PI / 8);
          modules[FL].speedMetersPerSecond = 0.02;
          modules[FR].speedMetersPerSecond = 0.02;
          modules[BR].speedMetersPerSecond = 0.02;
          modules[BL].speedMetersPerSecond = 0.02;
          setModuleStates(modules);
        });
  }

  //Aligns the limelight to have near 0 degrees horizontal offset (around 0 tx)
  public Command alignToTagCommand(Limelight lime) {

    PIDController rotationController = new PIDController(0.025, 0, 0.000033);
    rotationController.setSetpoint(0);
    rotationController.setTolerance(1);

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1, 0);

    return new FunctionalCommand(
        () -> {

        },
        () -> {
          if (lime.isTagFound()) {
            double tx = lime.getTX();

            SmartDashboard.putNumber("tx in alignToTagCommand", tx);

            drive(-feedforward.calculate(tx) + rotationController.calculate(tx), 0, 0, true);
          }
        },
        (_unused) -> {

        },
        rotationController::atSetpoint,
        this);
  }

  // Takes in a target distance to drive to away from the tag
  //Not using this in RobotContainer.java, using this in alignAndDriveToTagCommand()
  public Command driveToTagCommand(double targetDistance, Limelight lime) {

    PIDController driveController = new PIDController(0.395, 0, 0);
    driveController.setSetpoint(targetDistance);
    driveController.setTolerance(0.1);

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.05, 0);

    return new FunctionalCommand(
        () -> {

        },
        () -> {
          double distance = lime.getDistanceToTagInFeet();

          drive(0, -feedforward.calculate(distance) + driveController.calculate(distance), 0, false);
        },
        (_unused) -> {

        },
        driveController::atSetpoint,
        this);
  }

  //Finds the Closest Distances That We have Calibrated Shooting From
  // Not using this in RobotContainer.java, using this in alignAndDriveToTagCommand
  public double[] findClosestDistance(double currentDistance){
    double[] knownDistances = Constants.Limelight.knownDriveDistances;

    //Final Distance from Known Distances
    double closestDistance = 0.0;
    
    /*
     * Needs to be Maxiximum Value possible as it 
     * gets compared to smaller numbers in the code
     * below.
     */
    double closestDistanceFromKnownPoint = Double.MAX_VALUE;
    int index = Integer.MAX_VALUE;

    for(int i = 0; i < knownDistances.length; i++){

        double distanceFromKnownPoint = Math.abs(currentDistance - knownDistances[i]);
        if(distanceFromKnownPoint < closestDistanceFromKnownPoint) {
            closestDistanceFromKnownPoint = distanceFromKnownPoint;
            closestDistance = knownDistances[i];
            index = i;
        }
    }
    double returnValues[] = {closestDistance, index};
    return returnValues;
  }

  // For Speaker with various distances to shoot
  public Command alignAndDriveToTagCommand(Limelight lime) {

    PIDController rotationController = new PIDController(0.025, 0, 0.000033);
    rotationController.setSetpoint(0);
    rotationController.setTolerance(1);

    SimpleMotorFeedforward rotationFeedForward = new SimpleMotorFeedforward(0, 0);

    PIDController driveController = new PIDController(0.395, 0, 0);
    driveController.setTolerance(0.1);
    

    SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.01, 0);

    return new FunctionalCommand(
        () -> {
          
        },
        () -> {
          double distance = lime.getDistanceToTagInFeet();
          double toDriveDistance = 0;

          if (distance > 6) {
            toDriveDistance = 6;
          }
          else {
            toDriveDistance = distance;
          }

          driveController.setSetpoint(toDriveDistance);

          SmartDashboard.putNumber("finding closest distance", toDriveDistance);
          SmartDashboard.putNumber("distance", distance);

          double tx = lime.getTX();

          double driveSpeed = driveController.calculate(distance);

          if(driveSpeed >= 3.5) {
            driveSpeed = 3.5;
          }
          else if (driveSpeed <= -3.5) {
            driveSpeed = -3.5;
          }

          if (lime.isTagFound()) {
            drive(
                -rotationFeedForward.calculate(tx) + rotationController.calculate(tx),
                -driveFeedForward.calculate(distance) + driveSpeed,
                0,
                false);
          }
        },
        (_unused) -> {

        },
        () -> {
          return driveController.atSetpoint() && rotationController.atSetpoint();
        },
        this);
  }

  //Getting to the amp diagonally
  public Command alignAndGetPerpendicularToTagCommand(Limelight lime) {

    PIDController rotationController = new PIDController(0.0315, 0, 0.000033);
    rotationController.setSetpoint(0);
    rotationController.setTolerance(1);

    SimpleMotorFeedforward rotationFeedForward = new SimpleMotorFeedforward(0, 0);

    PIDController driveController = new PIDController(0.395, 0, 0);
    driveController.setSetpoint(0);
    driveController.setTolerance(0.1);

    SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.01, 0);

    return new FunctionalCommand(
        () -> {

        },
        () -> {
          double distance = lime.getDistanceToTagInFeet();

          double driveSpeed = driveController.calculate(distance);

          if(driveSpeed >= 3.5) {
            driveSpeed = 3.5;
          }
          else if (driveSpeed <= -3.5) {
            driveSpeed = -3.5;
          }

          if (lime.isTagFound()) {
            drive(
                rotationFeedForward.calculate(gyro.getYaw().getValueAsDouble() - 270) - rotationController.calculate(gyro.getYaw().getValueAsDouble() - 270),
                -driveFeedForward.calculate(distance) + driveSpeed,
                0,
                false);
          }
        },
        (_unused) -> {

        },
        () -> {
          return driveController.atSetpoint() && rotationController.atSetpoint();
        },
        this);
  }



  public Command xMode() {
    return runOnce(
        () -> {
          ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
          SwerveModuleState[] modules = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
          modules[FL].angle = new Rotation2d(Math.PI / 4.0);
          modules[FR].angle = new Rotation2d((-5.0 * Math.PI) / 4.0);
          modules[BR].angle = new Rotation2d((Math.PI) / 4.0);
          modules[BL].angle = new Rotation2d((-5.0 * Math.PI) / 4.0);
          modules[FL].speedMetersPerSecond = 0.02;
          modules[FR].speedMetersPerSecond = 0.02;
          modules[BR].speedMetersPerSecond = 0.02;
          modules[BL].speedMetersPerSecond = 0.02;
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

          // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0,0);
          // SwerveModuleState[] modules =
          // swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
          // setModuleStates(modules);
          this.stop();
        });
  }

  public void resetGyro() {
    // gyro.reset();
    gyroZero = gyro.getYaw().getValueAsDouble();
    odometry.resetPosition();
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
}
