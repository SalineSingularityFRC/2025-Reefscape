package frc.robot;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;

/*
 * This class should hold any static configuration data about the robot
 * All variables in this class should be marked public static and final
 */
public final class Constants {

  public static final class Analog {
    public static final class SwerveModule {
      public static final class Channel_ID {
        public static final int FL = 1;
      }
    }
  }

  public static final class CanId {
    public static final class Intake {
      public static final int LEFT_MOTOR = 33;
      public static final int RIGHT_MOTOR = 34;
      public static final int INTAKE_LASER = 49;
      public static final int SHOOTER_LASER = 32;
    }
    
    public static final class CanCoder {
      public static final int GYRO = 20;
      public static final int FL = 1;
      public static final int FR = 2;
      public static final int BL = 3;
      public static final int BR = 4;
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

  public static final class Position {
    public static final class MainArm {
      public static final class Auton {
        public static final class CloseNote {
          public static final double SIDE = 19; // Close Note1 or Close Note3
        }

        public static final class WhiteLine {
          public static final double SIDE = 18; // Not clearly tuned
          public static final double MIDDLE = 17.5; // Not clearly tuned
        }
      }

      public static final class Speaker {
          public static final double SIDE = 11; // Touching the right or left side of the Speaker
          public static final double MIDDLE = 10; // Middle of speaker
          public static final double FEET3 = 10; // 3 feet from april tag speaker
          public static final double FEET6 = 17.1162; // 6 feet from april tag speaker
          public static final double FEET7_4 = 18.48877; // 7.4 feet from april tag speaker
      }

      public static final double AMP = 48.5;
      public static final double PICKUP = 0.5;
      public static final double CLIMBER = 0;
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
    public static final String DEFAULT = "rio";
    public static final String DRIVE_TRAIN = "drivetrain";
  }

  public static final class LaserCan {
    public static final double INTAKE_WIDTH_MM = Measurement.INTAKE_WIDTH_M * 1000;
    public static final double INTAKE_TOLERANCE_MM_1 = 100; //Includes height of sensor and other factors
    public static final double INTAKE_TOLERANCE_MM_2 = 50; //Includes height of sensor and other factors

  }

  public static final class Gamepad {

    public static final class Controller {
      public static final int DRIVE = 0;
      public static final int ARM = 1;
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

  public static final class WheelOffset {
    // Converting rotations to radians
    public static final double FL = (0.962646) * 2 * Math.PI;
    public static final double FR = (0.682861) * 2 * Math.PI;
    public static final double BL = (0.760986) * 2 * Math.PI;
    public static final double BR = (0.555908) * 2 * Math.PI;
  }

  public static final class MotorGearRatio {
    public static final double DRIVE = 6.75;
    public static final double ANGLE = 150.0 / 7.0;// 12.8; 
                                                   // https://www.swervedrivespecialties.com/products/mk4-swerve-module
    public static final int BIG = 10;
    public static final double ARM = 45.0;
    public static final int SMALL = 7;
  }

  public static final class Measurement {
    // trackWidth - lateral distance between pairs of wheels on different sides of
    // the robot
    // wheelBase - distance between pairs of wheels on the same side of the robot
    // driveBaseRadius - distance from robot center to furthest module.
    // radiusFactor - to account for real world factors of the wheel radius
    // THIS IS IMPORTANT FOR A RECTANGULAR ROBOT
    // In meters
    public static final double TRACK_WIDTH = 18.75 * 0.0254; // Inches to meters
    public static final double WHEEL_BASE = 22.75 * 0.0254; // Inches to meters
    public static final double WHEELRADIUS = 2.003 * 0.0254; // 2024 robot radius from inches to meters
    public static final double DRIVEBASERADIUS = 14.942 * 0.0254; // Inches to meters
    public static final double INTAKE_WIDTH_M = Units.inchesToMeters(19.25);
  }

  public static final class Speed {
    public static final double ROBOT_SPEED_DIVISOR = 2.5; // what the max speed should be divided by, 1 is max power
    public static final double SHOOTER = 65; // speed of the shooter in rotations per second
    public static final double AMPSHOOTER = 15; // speed of the shooter in rotations per second
    public static final double INTAKE = 20; // rotations per second
    public static final double ARM = 30; // rotations per second
    public static final double HOME = 0.2; // proportional
    public static final double CLIMBER = 280;
    public static final double ARMDUTYCYCLEUP = 0.4; // between -1 and 1
    public static final double ARMDUTYCYCLEDOWN = 0.2; // between -1 and 1
    public static final double REVERSESHOOTER = 0.15; // between -1 and 1
  }

  public static final class Distance {
    // CHARGE STATION COMMUNITY DISTANCE:
    public static final double TO_BLUE_CHARGE_STATION = 96.4694981;
    public static final double TO_RED_CHARGE_STATION = 99;
    public static final double TO_CENTER_COMMUNITY = 100;
    public static final double TO_OUTSIDE_COMMUNITY = 87.30208217;
    // 1.832716884 is the number of inches per 1 encoder value
    // ~80 (plus offset) to the center of the charge station for robot
    // ~160 is the distance to leave the community plus some extra cushion
  }

  public static final class AngleInaccuracy {
    public static final double MAX = Math.PI / 24;
  }

  public static final class PidGains {
    public static final class PathPlanner {
      public static final PID translation = new PID(3, 5, 0.0);
      public static final PID rotation = new PID(1, 1, 0.3);
    }

    public static final class Limelight {
      public static final PID DRIVE_CONTROLLER = new PID(0.0025, 0, 0);
      public static final PID TURN_CONTROLLER = new PID(0.01,0,0);
      public static final PID SCORE_DRIVE_CONTROLLER = new PID(0.0056, 0, 0);
      
    }

    public static final class TurnAngle {
      public static final double[] TURN_ANGLE = { Math.PI / 6, 0, 0 };
    }

    public static final class SwerveModule {
      public static final PID DRIVE_PID_CONTROLLER = new PID(.5, 0, 0);
      public static final PID TURNING_PID_CONTROLLER = new PID(7, 0, 0.3, 0);
    }
  }

  public static final class Limelight {
    public static final double[] knownDriveDistances = {3, 6, 8};
    public static final double[] knownShootingPositions = {
      Constants.Position.MainArm.Speaker.FEET3, 
      Constants.Position.MainArm.Speaker.FEET6, 
      Constants.Position.MainArm.Speaker.FEET7_4
    };
  }

  public static final class Modes {
    /*
     * Mode.REAL - if on a real robot
     * Mode.SIM - if on "Simulate Robot Code"
     * Mode.REPLAY - finds path to log file and puts in AdvantageScope (if open)
    */
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM; // Mode.REAL : Mode.REPLAY;
  }

  public static enum Mode{REAL, SIM, REPLAY}

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
  }

  public static class Elevator {
    public static class Positions {
      public static ConfigInt FEED_STATION_COUNTS = new ConfigInt("Elevator/Positions/Feed Station in counts", 0);
      public static ConfigInt L1_COUNTS = new ConfigInt("Elevator/Positions/L1 in counts", 10);
      public static ConfigInt L2_COUNTS = new ConfigInt("Elevator/Positions/L2 in counts", 30);
      public static ConfigInt L3_COUNTS = new ConfigInt("Elevator/Positions/L3 in counts", 60);
      public static ConfigInt L4_COUNTS = new ConfigInt("Elevator/Positions/L4 in counts", 90);
    }
    
    public static class PrimaryMotor {
      public static ConfigInt CAN_ID = new ConfigInt("Elevator/Primary Motor/CAN ID", 40);
      public static ConfigDouble KP = new ConfigDouble("Elevator/Primary Motor/kP", 5);
  
      public static ConfigDouble MIN_POWER = new ConfigDouble("Elevator/Primary Motor/Min Power", -1);
      public static ConfigDouble MAX_POWER = new ConfigDouble("Elevator/Primary Motor/Max Power", 1);
  
      public static ConfigDouble MAX_VELOCITY_RPM = new ConfigDouble("Elevator/Primary Motor/Max Velocity in rpm", 2000);
      public static ConfigDouble MAX_ACCEL_RPM_PER_S = new ConfigDouble("Elevator/Primary Motor/Max Accel in rpm per s", 10000);
      public static ConfigInt MAX_CURRENT_IN_A = new ConfigInt("Elevator/Primary Motor/Max Current in A", 40);
      public static ConfigDouble VOLTAGE_COMPENSATION_IN_V = new ConfigDouble("Elevator/Primary Motor/Voltage Compensation in V", 12);
      public static ConfigDouble MAX_CONTROL_ERROR_IN_COUNTS = new ConfigDouble("Elevator/Primary Motor/Control Error Tolerance", 0.25);  
    }
  }

  public static class Intake{
    public static class Nums{
      public static ConfigDouble motorSpeed = new ConfigDouble("Intake Motor Speed", .1);
      public static ConfigDouble sensingDistance = new ConfigDouble("Sensor Max Distance", 100);
    }
    public static class LeftMotor{
      public static ConfigDouble KP = new ConfigDouble("Intake Left P", .1);
      public static ConfigDouble MAX_POWER = new ConfigDouble("Intake Left Max Power", 1);
      public static ConfigDouble MIN_POWER = new ConfigDouble("Intake Left Max Power", -1);
      public static ConfigDouble MAX_VELOCITY = new ConfigDouble("Intake Left Max V", 2000);
      public static ConfigDouble MAX_ACCELERATION = new ConfigDouble("Intake Left Max A", 10000);
      public static ConfigDouble MAX_CLOSED_LOOP_ERROR = new ConfigDouble("Intake Left Max Error", 0.25);
    }
    public static class RightMotor{
      public static ConfigDouble KP = new ConfigDouble("Intake Right P", .1);
      public static ConfigDouble MAX_POWER = new ConfigDouble("Intake Right Max Power", 1);
      public static ConfigDouble MIN_POWER = new ConfigDouble("Intake Right Max Power", -1);
      public static ConfigDouble MAX_VELOCITY = new ConfigDouble("Intake Right Max V", 2000);
      public static ConfigDouble MAX_ACCELERATION = new ConfigDouble("Intake Right Max A", 10000);
      public static ConfigDouble MAX_CLOSED_LOOP_ERROR = new ConfigDouble("Intake Right Max Error", 0.25);
    }
  }
}
