package frc.robot;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
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
      public static final int LEFT_MOTOR = 30;
      public static final int RIGHT_MOTOR = 31;
      public static final int INTAKE_LASER = 32;
      public static final int SHOOTER_LASER = 33;
    }

    public static final class Climber {
      public static final int MOTOR = 60;
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


  public static final class SwerveModule {

    /*
     * The following are for the Mk4n Swerve Modules, L2+ Ratio, FOC on (from liscence)
     */
    public static final class GearRatio {
      public static final double DRIVE = 5.9;
      public static final double ANGLE = 18.75;
    }

    // Free speed of the motors
    public static final class Speed {
      public static final double MAX_SPEED = Units.feetToMeters(17.1); // 5.21208 m/s
    }

    public static final class WheelOffset {
      public static final double FL = Units.rotationsToRadians(0.972168);
      public static final double FR = Units.rotationsToRadians(0.679932);
      public static final double BL = Units.rotationsToRadians(0.760742);
      public static final double BR = Units.rotationsToRadians(0.548828);
    }

  }

  public static final class Measurement {
    // trackWidth - lateral distance between pairs of wheels on different sides of
    // the robot
    // wheelBase - distance between pairs of wheels on the same side of the robot
    // driveBaseRadius - distance from robot center to furthest module.
    // radiusFactor - to account for real world factors of the wheel radius

    public static final double TRACK_WIDTH = Units.inchesToMeters(18.75);
    public static final double WHEEL_BASE = Units.inchesToMeters(22.75);
    public static final double WHEELRADIUS = Units.inchesToMeters(1.8787);
    public static final double INTAKE_WIDTH_M = Units.inchesToMeters(19.25);
  }

  public static final class AngleInaccuracy {
    public static final double MAX = Math.PI / 24;
  }

  public static final class PidGains {
    public static final class PathPlanner {
      //public static final PID translation = new PID(3, 5, 0.0);
      public static final PID translation = new PID(3, 0, 0.011);
      //public static final PID rotation = new PID(1, 0, 0.3);
      public static final PID rotation = new PID(3, 0, 0);
    }

    public static final class rotationCorrection {
      public static final PID rotation = new PID(0.22, 0, 0);
    }


    public static final class Limelight {
      public static final PID DRIVE_CONTROLLER = new PID(0.0025, 0, 0);
      public static final PID TURN_CONTROLLER = new PID(0.01,0,0);
      public static final PID SCORE_DRIVE_CONTROLLER = new PID(0.0056, 0, 0);
      
    }

    public static final class SwerveModule {
      //On real carpet
      public static final PID DRIVE_PID_CONTROLLER = new PID(5, 0, 0, 2.5);
      //public static final PID TURNING_PID_CONTROLLER = new PID(7, 0, 0.1, 0);

      // On real carpet
      public static final PID TURNING_PID_CONTROLLER = new PID(7, 0, 0.1,0.2);

    }
  }

  public static final class PathplannerConfig {
    public static Translation2d[] ChassisModuleOffsets = {
      new Translation2d(0.289, 0.238), 
      new Translation2d(0.289, -0.238),
      new Translation2d(-0.289, 0.238),
      new Translation2d(-0.289, -0.238),
    };

    public static ModuleConfig ChassisModuleConfig = 
      new ModuleConfig(
        Measurement.WHEELRADIUS, 
        SwerveModule.Speed.MAX_SPEED, 
        1.0, 
        null, 
        SwerveModule.GearRatio.DRIVE, 
        60, 
        1);
        
    public static RobotConfig ChassisRobotConfig = 
      new RobotConfig(23.350, 1.705, ChassisModuleConfig, ChassisModuleOffsets);
  }

  // public static final class Limelight {
  //   public static final double[] knownDriveDistances = {3, 6, 8};
  //   public static final double[] knownShootingPositions = {
  //     Constants.Position.MainArm.Speaker.FEET3, 
  //     Constants.Position.MainArm.Speaker.FEET6, 
  //     Constants.Position.MainArm.Speaker.FEET7_4
  //   };
  // }

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

    public boolean isTrue(){
      return getValue() != 0;
    }

    public boolean isFalse(){
      return (!isTrue());
    }
  }
  public static class Climber {
    public static ConfigDouble WINCH_SPEED = new ConfigDouble("Climber/WINCH_SPEED", .5);


  }
  public static class Elevator {
    public static ConfigInt FOLLOW_DUALENABLE = new ConfigInt("Elevator/DUALENABLE", 0);

    public static class Positions {
      public static ConfigInt FEED_STATION_COUNTS = new ConfigInt("Elevator/Positions/Feed Station in counts", 0);
      public static ConfigInt L1_COUNTS = new ConfigInt("Elevator/Positions/L1 in counts", 10);
      public static ConfigInt L2_COUNTS = new ConfigInt("Elevator/Positions/L2 in counts", 30);
      public static ConfigInt L3_COUNTS = new ConfigInt("Elevator/Positions/L3 in counts", 60);
      public static ConfigInt L4_COUNTS = new ConfigInt("Elevator/Positions/L4 in counts", 90);
    }
    
    public static class PrimaryMotor {
      public static ConfigDouble RAISE_SPEED = new ConfigDouble("Elevator/Primary Motor/RAISE_SPEED", 0.1);
      public static ConfigDouble LOWER_SPEED = new ConfigDouble("Elevator/Primary Motor/LOWER_SPEED", .1);
      public static ConfigInt INVERTED = new ConfigInt("Elevator/Primary Motor/ INVERTED MOTOR", 1);
      public static ConfigInt CAN_ID = new ConfigInt("Elevator/Primary Motor/CAN ID", 40);
      public static ConfigDouble KP = new ConfigDouble("Elevator/Primary Motor/kP", 5);
      public static ConfigDouble KD = new ConfigDouble("Elevator/Primary Motor/kD", 0);
  
      public static ConfigDouble MIN_POWER = new ConfigDouble("Elevator/Primary Motor/Min Power", -1);
      public static ConfigDouble MAX_POWER = new ConfigDouble("Elevator/Primary Motor/Max Power", 1);
  
      public static ConfigDouble MAX_VELOCITY_RPM = new ConfigDouble("Elevator/Primary Motor/Max Velocity in rpm", 2000);
      public static ConfigDouble MAX_ACCEL_RPM_PER_S = new ConfigDouble("Elevator/Primary Motor/Max Accel in rpm per s", 200);
      public static ConfigInt MAX_CURRENT_IN_A = new ConfigInt("Elevator/Primary Motor/Max Current in A", 40);
      public static ConfigDouble VOLTAGE_COMPENSATION_IN_V = new ConfigDouble("Elevator/Primary Motor/Voltage Compensation in V", 12);
      public static ConfigDouble MAX_CONTROL_ERROR_IN_COUNTS = new ConfigDouble("Elevator/Primary Motor/Control Error Tolerance", 0.25);  
    }

    public static class SecondaryMotor{
      public static ConfigInt CAN_ID = new ConfigInt("Elevator/Secondary Motor/CAN ID", 41);
    }
  }
}
