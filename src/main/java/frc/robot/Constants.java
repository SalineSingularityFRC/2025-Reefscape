package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;

/*
 * This class should hold any static configuration data about the robot
 * All variables in this class should be marked public static and final
 */
public final class Constants {

  public static final class CanId {
    public static final class Intake {
      public static final int LEFT_MOTOR = 30;
      public static final int RIGHT_MOTOR = 31;
      public static final int INTAKE_LASER = 32;
      public static final int SHOOTER_LASER = 33;
      public static final int TROUGH_LASER = 34;
    }

    public static final class Climber {
      public static final int MOTOR = 60;
    }

    public static final class Trougth {
      public static final int TROUGH_MOTOR = 62;
    }

    public static final class Algae {
      public static final int MAIN_MOTOR = 51;
      public static final int ALGAE_MOTOR = 50;
      public static final int ALGAE_LASER = 52;
    }

    public static final class Processor {
      public static final int INTAKE_MOTOR = 62;
    }

    public static final class CanCoder {
      public static final int GYRO = 20;
      public static final int FL = 1;
      public static final int FR = 2;
      public static final int BL = 3;
      public static final int BR = 4;
      public static final int ALGAE = 9;
    }

    public static final class Swerve {
      public static final class Angle {
        public static final int FL = 18;
        public static final int FR = 17;
        public static final int BL = 13;
        public static final int BR = 15;
      }

      public static final class Drive {
        public static final int FL = 12;
        public static final int FR = 11;
        public static final int BL = 16;
        public static final int BR = 14;
      }
    }

    public static final class LaserCan {
      public static final int SENSOR1 = 50;
      public static final int SENSOR2 = 49;
    }
  }

  public static final class Inverted {
    // This is for motors
    public static final boolean FL = false;
    public static final boolean FR = true;
    public static final boolean BL = false;
    public static final boolean BR = true;
    public static final boolean ANGLE = true;
  }

  public static final class Canbus {
    // public static final String DEFAULT = "rio";
    public static final String DRIVE_TRAIN = "drivetrain";
  }

  public static final class Gamepad {

    public static final class Controller {
      public static final int DRIVE = 0;
      public static final int BUTTON = 1;
      public static final int THIRD = 2;
    }

    public static final class Axis {
      public static final int LEFT_X = 0;
      public static final int LEFT_Y = 1;
      public static final int RIGHT = 4;
    }

    public static final class Trigger {
      public static final int LEFT = 2;
      public static final int RIGHT = 3;
    }

    public static final class Button {
      public static final int A = 1;
      public static final int B = 2;
      public static final int X = 3;
      public static final int Y = 4;
      public static final int LEFT = 5;
      public static final int RIGHT = 6;
      public static final int BACK = 7;
      public static final int START = 8;
      public static final int L_JOYSTICK = 9;
      public static final int R_JOYSTICK = 10;
    }
  }

  public static final class SwerveModule {

    /*
     * The following are for the Mk4n Swerve Modules, L2+ Ratio, FOC on (from
     * liscence)
     */
    public static final class GearRatio {
      public static final double DRIVE = 5.9;
      public static final double ANGLE = 18.75;
    }

    public static final class Speed {

      // Tested max speed in m/s
      public static final double MAX_SPEED = 4.865;
    }

    public static final class WheelOffset {
      public static final double FL = Units.rotationsToRadians(0.732666);
      public static final double FR = Units.rotationsToRadians(0.871094);
      public static final double BL = Units.rotationsToRadians(0.832764);
      public static final double BR = Units.rotationsToRadians(0.669189);
    }

  }

  public static final class Measurement {
    // trackWidth - lateral distance between pairs of wheels on different sides of
    // the robot
    // wheelBase - distance between pairs of wheels on the same side of the robot
    // WHEELRADIUS - Vex grip lock wheels: https://www.vexrobotics.com/217-9042.html

    public static final double TRACK_WIDTH = Units.inchesToMeters(21.75);
    public static final double WHEEL_BASE = Units.inchesToMeters(21.75);
    public static final double WHEELRADIUS = Units.inchesToMeters(2.0);

    // Without bumpers (meters)
    public static final double TOTAL_LENGTH = 0.70485;
    public static final double TOTAL_WIDTH = 0.6858;

    // Offest of limelight from robot center (inches)
    public static final double LIMELIGHT_BOTTOM_FORWARD = 12; // 0.3048 meters
    public static final double LIMELIGHT_BOTTOM_UP = 8.5; // 0.2159 meters
    public static final double LIMELIGHT_BOTTOM_RIGHT = 6.5; // 0.1651 meters
  }

  public static final class AngleInaccuracy {
    public static final double MAX = Math.PI / 24;
  }

  public static final class PidGains {
    public static final class PathPlanner {
      // public static final PID translation = new PID(3, 5, 0.0);
      public static final PID translation = new PID(3, 0, 0.011);
      // public static final PID rotation = new PID(1, 0, 0.3);
      public static final PID rotation = new PID(3, 0, 0);
    }

    public static final class rotationCorrection {
      public static final PID rotation = new PID(0.085, 0, 0);
    }

    public static final class Limelight {
      public static final PID DRIVE_CONTROLLER = new PID(0.0025, 0, 0);
      public static final PID TURN_CONTROLLER = new PID(0.01, 0, 0);
      public static final PID SCORE_DRIVE_CONTROLLER = new PID(0.0056, 0, 0);

    }

    public static final class SwerveModule {
      // On test carpet
      public static final PID DRIVE_PID_CONTROLLER = new PID(5.3, 0, 0.053, 3.5);
      // public static final PID TURNING_PID_CONTROLLER = new PID(7, 0, 0.1, 0);

      // On test carpet
      public static final PID TURNING_PID_CONTROLLER = new PID(7, 0, 0.1, 0.4);

    }
  }

  public static final class PathplannerConfig {
    public static Translation2d[] ChassisModuleOffsets = {
        new Translation2d(0.289, 0.238),
        new Translation2d(0.289, -0.238),
        new Translation2d(-0.289, 0.238),
        new Translation2d(-0.289, -0.238),
    };

    public static ModuleConfig ChassisModuleConfig = new ModuleConfig(
        Measurement.WHEELRADIUS,
        SwerveModule.Speed.MAX_SPEED,
        1.0,
        null,
        SwerveModule.GearRatio.DRIVE,
        60,
        1);

    public static RobotConfig ChassisRobotConfig = new RobotConfig(23.350, 1.705, ChassisModuleConfig,
        ChassisModuleOffsets);
  }

  public static final class Modes {
    /*
     * Mode.REAL - if on a real robot
     * Mode.SIM - if on "Simulate Robot Code"
     * Mode.REPLAY - finds path to log file and puts in AdvantageScope (if open)
     */
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM; // Mode.REAL : Mode.REPLAY;
  }

  public static enum Mode {
    REAL, SIM, REPLAY
  }

  public abstract static class Config<T> {
    public final String name;
    public final T defaultValue;

    Config(String name, T defaultValue) {
      this.name = name;
      this.defaultValue = defaultValue;
    }

    abstract public T getValue();
  }

  public static class ConfigDouble extends Config<Double> {
    public ConfigDouble(String name, double defaultValue) {
      super(name, defaultValue);

      // Make sure that it shows up in the Preferences
      Preferences.initDouble(name, defaultValue);
    }

    @Override
    public Double getValue() {
      return Preferences.getDouble(name, defaultValue);
    }
  }

  public static class ConfigInt extends Config<Integer> {
    public ConfigInt(String name, int defaultValue) {
      super(name, defaultValue);

      // Make sure that it shows up in the Preferences
      Preferences.initInt(name, defaultValue);
    }

    @Override
    public Integer getValue() {
      return Preferences.getInt(name, defaultValue);
    }

    public boolean isTrue() {
      return getValue() != 0;
    }

    public boolean isFalse() {
      return (!isTrue()); 
    }
  }

  public static class ConfigBoolean extends Config<Boolean> {
    public ConfigBoolean(String name, boolean defaultValue){
      super(name, defaultValue);

      Preferences.initBoolean(name, defaultValue);
    }

    public Boolean getValue(){
      return Preferences.getBoolean(name, defaultValue);
    }
  }

  public static class Vision {
    public static final Vector<N3> kDefaultSingleTagStdDevs = VecBuilder.fill(0.7, 0.7,9999999);
    public static final Vector<N3> kDefaultMultiTagStdDevs = VecBuilder.fill(0.5, 0.5,9999999);
    public static final ConfigDouble kKalmanDivisor = new ConfigDouble("Vision/kKalmanDivisor", 30);
    public static final ConfigDouble kMaxRotationRate = new ConfigDouble("Vision/kMaxRotationRate", 360);

    public class Names {
      // Switched top and left IRL (need to rename)
      public static final String leftLL = "limelight-left";
      public static final String rightLL = "limelight-right";
      public static final String realSenseCam = "real-sense-camera";
    }
  }

  public static class LED {
    public static ConfigDouble PWM_VALUE = new ConfigDouble("LED/PWM_VALUE", 0.15);
  }

  public static class Climber {
    public static ConfigDouble WINCH_SPEED = new ConfigDouble("Climber/WINCH_SPEED", .5);
    public static final ConfigDouble ENCODER_MAX_POS = new ConfigDouble("Climber/ENCODER_MAX_POS", 100);
    public static final ConfigDouble ENCODER_MIN_POS = new ConfigDouble("Climber/ENCODER_MIN_POS", -100);
  }

  public static class Trough {
    public static ConfigDouble TROUGH_SPEED = new ConfigDouble("Trough/TROUGH_SPEED", .05);
    public static final ConfigDouble ENCODER_MAX_POS = new ConfigDouble("Trough/ENCODER_MAX_POS", 1.0);
    public static final ConfigDouble ENCODER_MIN_POS = new ConfigDouble("Trough/ENCODER_MIN_POS", 0);
    public static final ConfigDouble HOME_POSITION = new ConfigDouble("Trough/ENCODER_HOME_POS", 0);
    public static final ConfigDouble CLIMB_POSITION = new ConfigDouble("Trough/ENCODER_CLIMB_POS", 1.0);
    public static final ConfigDouble KP = new ConfigDouble("Trough/KP", 1.0);
    public static final ConfigDouble KD = new ConfigDouble("Trough/KD", 0);
  }

  public static class Drive {
    public static ConfigInt PID_DRIVE_TUNING = new ConfigInt("Drive/PID/Tuning Enabled", 0);

    public static ConfigDouble PID_DRIVE_ROTATION_KP = new ConfigDouble("Drive/PID/Rotation kP", 0.1);
    public static ConfigDouble PID_DRIVE_ROTATION_KI = new ConfigDouble("Drive/PID/Rotation kI", 0);
    public static ConfigDouble PID_DRIVE_ROTATION_KD = new ConfigDouble("Drive/PID/Rotation kD", 0);
    public static ConfigDouble PID_DRIVE_ROTATION_TOLERANCE = new ConfigDouble("Drive/PID/Rotation Tolerance", 1);

    public static ConfigDouble PID_DRIVE_X_KP = new ConfigDouble("Drive/PID/X kP", 0.1);
    public static ConfigDouble PID_DRIVE_X_KI = new ConfigDouble("Drive/PID/X kI", 0);
    public static ConfigDouble PID_DRIVE_X_KD = new ConfigDouble("Drive/PID/X kD", 0);
    public static ConfigDouble PID_DRIVE_X_TOLERANCE = new ConfigDouble("Drive/PID/X Tolerance", 1);

    public static ConfigDouble PID_DRIVE_Y_KP = new ConfigDouble("Drive/PID/Y kP", 0.1);
    public static ConfigDouble PID_DRIVE_Y_KI = new ConfigDouble("Drive/PID/Y kI", 0);
    public static ConfigDouble PID_DRIVE_Y_KD = new ConfigDouble("Drive/PID/Y kD", 0);
    public static ConfigDouble PID_DRIVE_Y_TOLERANCE = new ConfigDouble("Drive/PID/Y Tolerance", 1);
    
    public static ConfigDouble PID_DRIVE_MAX_ROTATION_SPEED = new ConfigDouble("Drive/PID/Max Rotation", 2);
    public static ConfigDouble PID_DRIVE_MAX_DRIVE_X_SPEED = new ConfigDouble("Drive/PID/Max Drive X", 2);
    public static ConfigDouble PID_DRIVE_MAX_DRIVE_Y_SPEED = new ConfigDouble("Drive/PID/Max Drive Y", 2);

    public static ConfigDouble L4_PID_DRIVE_POSE_TOLERANCE =  new ConfigDouble("L4PIDDrive/Drive Pose Tolerance", 0.1);
    public static ConfigDouble L4_PID_DRIVE_STABLE_COUNT_THRESHOLD =  new ConfigDouble("L4PIDDrive/Stable Count Threshold", 8);
    public static ConfigDouble L4_PID_DRIVE_ROBOT_DISTANCE_TO_REEF = new ConfigDouble("L4PIDDrive/Reef to Robot Distance", 1);
  }

  public static class Algae {
    // No algae inside PIDs
    public static final ConfigDouble kPMain = new ConfigDouble("Algae/kPMain", 1);
    public static final ConfigDouble kDMain = new ConfigDouble("Algae/kDMain", 0);
    public static final ConfigDouble kSMain = new ConfigDouble("Algae/kSMain", 2.5);
    public static final ConfigDouble kVMain = new ConfigDouble("Algae/kVMain", 0.2);
    
    // Algae inside PIDs
    public static final ConfigDouble kPMainAlgaeInside = new ConfigDouble("Algae/kPMainAlgaeInside", 200);
    public static final ConfigDouble kIMainAlgaeInside = new ConfigDouble("Algae/kIMainAlgaeInside", 1);
    public static final ConfigDouble kDMainAlgaeInside = new ConfigDouble("Algae/kDMainAlgaeInside", 0);
    public static final ConfigDouble kSMainAlgaeInside = new ConfigDouble("Algae/kSMainAlgaeInside", 2.5);
    public static final ConfigDouble kVMainAlgaeInside = new ConfigDouble("Algae/kVMainAlgaeInside", 2.0);

    public static final ConfigDouble kPAlgae = new ConfigDouble("Algae/kPAlgae", 1);
    public static final ConfigDouble kDAlgae = new ConfigDouble("Algae/kDAlgae", 0);
    public static final ConfigDouble kP1Algae = new ConfigDouble("Algae/kP1Algae", 5);
    public static final ConfigDouble kD1Algae = new ConfigDouble("Algae/kD1Algae", 0);

    public static final ConfigDouble sensingDistance = new ConfigDouble("Algae/SensingDistance", 100);
    public static ConfigDouble manualSpeed = new ConfigDouble("Algae/ManualControlSpeed", 10);
    public static ConfigDouble motorSpeedSlow = new ConfigDouble("Algae/SpeedSlow", 70);
    public static ConfigDouble motorSpeedFast = new ConfigDouble("Algae/SpeedFast", -60);
    public static ConfigDouble INTAKE_POS = new ConfigDouble("Algae/IntakePos", 20);
    public static ConfigDouble CORAL_SCORE_POSE = new ConfigDouble("Algae/Coral Score Pose", 3.916);
    public static ConfigDouble DEFAULT_POSE = new ConfigDouble("Algae/Default Pose", 0);
    public static ConfigDouble SHOOT_POS = new ConfigDouble("Algae/ShootPos", 40);
    public static ConfigDouble MAX_CONTROL_ERROR_IN_COUNTS = new ConfigDouble("Algae/Control Error Tolerance", 0.25);
  }

  public static class Processor {
    public static ConfigDouble intakeSpeed = new ConfigDouble("Processor/intakeSpeed", 30);
    public static ConfigDouble spitSpeed = new ConfigDouble("Processor/spitSpeed", -30);
    public static ConfigDouble kP = new ConfigDouble("Processor/kP", 1);
    public static class MotorStuff {
      public static ConfigDouble MIN_POWER = new ConfigDouble("Elevator/Primary Motor/Min Power", -1);
      public static ConfigDouble MAX_POWER = new ConfigDouble("Elevator/Primary Motor/Max Power", 1);

      public static ConfigDouble MAX_VELOCITY_RPM = new ConfigDouble("Elevator/Primary Motor/Max Velocity in rpm",
          2000);
      public static ConfigDouble MAX_ACCEL_RPM_PER_S = new ConfigDouble("Elevator/Primary Motor/Max Accel in rpm per s",
          200);
      public static ConfigInt MAX_CURRENT_IN_A = new ConfigInt("Elevator/Primary Motor/Max Current in A", 40);
      public static ConfigDouble VOLTAGE_COMPENSATION_IN_V = new ConfigDouble(
          "Elevator/Primary Motor/Voltage Compensation in V", 12);
      public static ConfigDouble MAX_CONTROL_ERROR_IN_COUNTS = new ConfigDouble(
          "Elevator/Primary Motor/Control Error Tolerance", 0.25);
    }
  }

  public static class Elevator {

    public static ConfigInt FOLLOW_DUALENABLE = new ConfigInt("Elevator/DUALENABLE", 0);

    public static class Heights {

      // From the ground (inches)
      public static ConfigDouble LOWEST_HEIGHT = new ConfigDouble("Elevator/Heights/Lowest Height", 12);
      public static ConfigDouble HIGHEST_HEIGHT = new ConfigDouble("Elevator/Positions/Highest Height", 72);
      public static ConfigDouble DEADZONE = new ConfigDouble("Elevator/Positions/Deadzone Intake See Elevator", 7.0);
    }

    public static class Positions {
      public static ConfigDouble FEED_STATION_COUNTS = new ConfigDouble("Elevator/Positions/Feed Station in counts", -2);
      public static ConfigDouble L1_COUNTS = new ConfigDouble("Elevator/Positions/L1 in counts", -2);
      public static ConfigDouble L2_COUNTS = new ConfigDouble("Elevator/Positions/L2 in counts", 23);
      public static ConfigDouble L3_COUNTS = new ConfigDouble("Elevator/Positions/L3 in counts", 53);
      public static ConfigDouble L4_COUNTS = new ConfigDouble("Elevator/Positions/L4 in counts", 97.5);
      public static ConfigDouble L4_COUNTS_ADDITIONAL_RAISE = new ConfigDouble("Elevator/Positions/L4 additional raise in counts", 1);
    }

    public static class PrimaryMotor {
      public static ConfigDouble RAISE_SPEED = new ConfigDouble("Elevator/Primary Motor/RAISE_SPEED", 0.3);
      public static ConfigDouble L4_RAISE_SPEED = new ConfigDouble("Elevator/Primary Motor/L4_RAISE_SPEED", 0.1);
      public static ConfigDouble LOWER_SPEED = new ConfigDouble("Elevator/Primary Motor/LOWER_SPEED", 0.2);
      public static ConfigInt INVERTED = new ConfigInt("Elevator/Primary Motor/ INVERTED MOTOR", 1);
      public static ConfigInt CAN_ID = new ConfigInt("Elevator/Primary Motor/CAN ID", 40);
      public static ConfigDouble KPUP = new ConfigDouble("Elevator/Primary Motor/kPUP", 0.082);
      public static ConfigDouble KIUP = new ConfigDouble("Elevator/Primary Motor/kIUP", 0);
      public static ConfigDouble KDUP = new ConfigDouble("Elevator/Primary Motor/kDUP", 0);
      public static ConfigDouble KFUP = new ConfigDouble("Elevator/Primary Motor/kFUP", 0);
      public static ConfigDouble KPDOWN = new ConfigDouble("Elevator/Primary Motor/kPDOWN", 0.073);
      public static ConfigDouble KIDOWN = new ConfigDouble("Elevator/Primary Motor/kIDOWN", 0);
      public static ConfigDouble KDDOWN = new ConfigDouble("Elevator/Primary Motor/kDDOWN", 0);
      public static ConfigDouble KFDOWN = new ConfigDouble("Elevator/Primary Motor/kFDOWN", 0);
      public static ConfigDouble arbFF = new ConfigDouble("Elevator/Primary Motor/arbFF", 0);
  
      public static ConfigDouble MIN_POWER = new ConfigDouble("Elevator/Primary Motor/Min Power", -1);
      public static ConfigDouble MAX_POWER = new ConfigDouble("Elevator/Primary Motor/Max Power", 1);
  
      public static ConfigDouble MAX_VELOCITY_RPM = new ConfigDouble("Elevator/Primary Motor/Max Velocity in rpm", 6784);
      public static ConfigDouble MAX_ACCEL_RPM_PER_S = new ConfigDouble("Elevator/Primary Motor/Max Accel in rpm per s", 20000);
      public static ConfigInt MAX_CURRENT_IN_A = new ConfigInt("Elevator/Primary Motor/Max Current in A", 60);
      public static ConfigDouble VOLTAGE_COMPENSATION_IN_V = new ConfigDouble("Elevator/Primary Motor/Voltage Compensation in V", 12);
      public static ConfigDouble MAX_CONTROL_ERROR_IN_COUNTS = new ConfigDouble("Elevator/Primary Motor/Control Error Tolerance", 0.25);  
    }

    public static class SecondaryMotor {
      public static ConfigInt CAN_ID = new ConfigInt("Elevator/Secondary Motor/CAN ID", 41);
    }
  }

  public static class Intake {
    public static class Nums {
      public static ConfigDouble conveyorSpeed = new ConfigDouble("Conveyor Motor Speed", .5);
      public static ConfigDouble motorSpeed = new ConfigDouble("Intake Motor Speed", 70);
      public static ConfigDouble motorSpeedSlow = new ConfigDouble("Intake Motor Speed Slow", 35);
      public static ConfigDouble shooterSpeed = new ConfigDouble("Shooter Motor Speed", 65);
      public static ConfigDouble intakeDistance = new ConfigDouble("Intake Sensor Min Distance", 100);
      public static ConfigDouble shooterDistance = new ConfigDouble("Shooter Sensor Min Distance", 100);
      public static ConfigDouble troughSenserDistance = new ConfigDouble("Trough Sensor Min Distance", 150);
      public static ConfigBoolean laserCanLogicOverride = new ConfigBoolean("IMPORTANT LaserCan / Elevator Override", false);
    }

    public static class LeftMotor {
      public static ConfigDouble KP = new ConfigDouble("Intake Left P", 1.5);
      public static ConfigDouble KS = new ConfigDouble("Intake Left kS", 2.5);
      public static ConfigDouble KV = new ConfigDouble("Intake Left kV", 1);
      public static ConfigDouble MAX_POWER = new ConfigDouble("Intake Left Max Power", 1);
      public static ConfigDouble MIN_POWER = new ConfigDouble("Intake Left Max Power", -1);
      public static ConfigDouble MAX_VELOCITY = new ConfigDouble("Intake Left Max V", 2000);
      public static ConfigDouble MAX_ACCELERATION = new ConfigDouble("Intake Left Max A", 10000);
      public static ConfigDouble MAX_CLOSED_LOOP_ERROR = new ConfigDouble("Intake Left Max Error", 0.25);
    }

    public static class RightMotor {
      public static ConfigDouble KP = new ConfigDouble("Intake Right P", 1.5);
      public static ConfigDouble KS = new ConfigDouble("Intake Right kS", 2.5);
      public static ConfigDouble KV = new ConfigDouble("Intake Right kV", 1);
      public static ConfigDouble MAX_POWER = new ConfigDouble("Intake Right Max Power", 1);
      public static ConfigDouble MIN_POWER = new ConfigDouble("Intake Right Max Power", -1);
      public static ConfigDouble MAX_VELOCITY = new ConfigDouble("Intake Right Max V", 2000);
      public static ConfigDouble MAX_ACCELERATION = new ConfigDouble("Intake Right Max A", 10000);
      public static ConfigDouble MAX_CLOSED_LOOP_ERROR = new ConfigDouble("Intake Right Max Error", 0.25);
    }
  }
}
