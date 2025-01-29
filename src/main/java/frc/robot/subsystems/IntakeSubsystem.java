package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Preferences;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax rightMotor;
    private SparkMax leftMotor;
    private LaserCan intakeSensor;
    private LaserCan shooterSensor;
    private double motorSpeed;
    private double sensingDistance;
    public static final SparkMaxConfig intakeLeftConfig = new SparkMaxConfig();
    public static final SparkMaxConfig intakeRightConfig = new SparkMaxConfig();
        private static final int LASER_CAN_NO_MEASUREMENT = -1;
    
        public IntakeSubsystem() {
            intakeSensor = new LaserCan(Constants.CanId.Intake.INTAKE_LASER);
            shooterSensor = new LaserCan(Constants.CanId.Intake.SHOOTER_LASER);
    
            sensingDistance = Preferences.getDouble("Sensing Distance", 100);
            motorSpeed = Preferences.getDouble("Intake Motor Speed", 1);
    
    
            // Left Motor
            double intakeLeftConfigP = Preferences.getDouble("Intake Left P", .1);
            double intakeLeftConfigPowerMax = Preferences.getDouble("Intake Left Max Power", 1);
            double intakeLeftConfigPowerMin = Preferences.getDouble("Intake Left Max Power", -1);
            double intakeLeftConfigMaxV = Preferences.getDouble("Intake Left Max V", 2000);
            double intakeLeftConfigMaxA = Preferences.getDouble("Intake Left Max A", 10000);
            double intakeLeftConfigMaxClosedLoopError = Preferences.getDouble("Intake Left Max Error", 0.25);
            intakeLeftConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
            intakeLeftConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Set PID values for position control
                    .p(intakeLeftConfigP)
                    .outputRange(intakeLeftConfigPowerMin, intakeLeftConfigPowerMax).maxMotion
                    // Set MAXMotion parameters for position control
                    .maxVelocity(intakeLeftConfigMaxV)
                    .maxAcceleration(intakeLeftConfigMaxA)
                    .allowedClosedLoopError(intakeLeftConfigMaxClosedLoopError);
            leftMotor = new SparkMax(Constants.CanId.Intake.LEFT_MOTOR, MotorType.kBrushless);
            leftMotor.configure(
                    intakeLeftConfig,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
    
            // Right Motor
            double intakeRightConfigP = Preferences.getDouble("Intake Right P", .1);
            double intakeRightConfigPowerMax = Preferences.getDouble("Intake Right Max Power", 1);
            double intakeRightConfigPowerMin = Preferences.getDouble("Intake Right Max Power", -1);
            double intakeRightConfigMaxV = Preferences.getDouble("Intake Right Max V", 2000);
            double intakeRightConfigMaxA = Preferences.getDouble("Intake Right Max A", 10000);
            double intakeRightConfigMaxClosedLoopError = Preferences.getDouble("Intake Right Max Error", 0.25);
            intakeRightConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
            intakeRightConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Set PID values for position control
                    .p(intakeRightConfigP)
                    .outputRange(intakeRightConfigPowerMin, intakeRightConfigPowerMax).maxMotion
                    // Set MAXMotion parameters for position control
                    .maxVelocity(intakeRightConfigMaxV)
                    .maxAcceleration(intakeRightConfigMaxA)
                    .allowedClosedLoopError(intakeRightConfigMaxClosedLoopError);
            rightMotor = new SparkMax(Constants.CanId.Intake.RIGHT_MOTOR, MotorType.kBrushless);
            rightMotor.configure(
                    intakeLeftConfig,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
        }
    
        public void periodic() {
            SmartDashboard.putNumber("Intake Sensor", getSensorValue(intakeSensor));
            SmartDashboard.putNumber("Shooter Sensor", getSensorValue(shooterSensor));
        }
    
        public boolean canSeeCoral(LaserCan sensor) {
            // TODO: Handle no sensor reading.
            if (sensor.getMeasurement() != null && getSensorValue(sensor) <= sensingDistance) {
                return true;
            }
            return false;
        }
    
        public boolean coralInIntake() {
            return canSeeCoral(intakeSensor);
        }
    
        public boolean coralInShooter() {
            return canSeeCoral(shooterSensor);
        }
    
        public int getSensorValue(LaserCan sensor) {
            Measurement measurement = sensor.getMeasurement();
            return measurement == null ? LASER_CAN_NO_MEASUREMENT : measurement.distance_mm;
    }

    public Command runMotors() {
        return runEnd(
                () -> {
                    leftMotor.set(-motorSpeed);
                    rightMotor.set(-motorSpeed);
                },
                () -> {
                    leftMotor.stopMotor();
                    rightMotor.stopMotor();
                });
    }

    public Command shootCoral() {
        return runMotors().until(() -> !coralInShooter());
    }

    public Command intakeCoral() {
        return runMotors().until(() -> coralInShooter());
    }
}
