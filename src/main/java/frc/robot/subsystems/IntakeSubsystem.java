package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import edu.wpi.first.wpilibj.Preferences;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSubsystem extends SubsystemBase {
    private SparkFlex rightMotor;
    private SparkFlex leftMotor;
    private LaserCan intakeSensor;
    private double motorSpeed;
    private double sensingDistance;
    public static final SparkFlexConfig intakeLeftConfig = new SparkFlexConfig();
    public static final SparkFlexConfig intakeRightConfig = new SparkFlexConfig();
        private static final int LASER_CAN_NO_MEASUREMENT = -1;
    
        public IntakeSubsystem() {
            intakeSensor = new LaserCan(Constants.CanId.Intake.INTAKE_LASER);
    
            // Left Motor
            intakeLeftConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
            intakeLeftConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Set PID values for position control
                    .p(Intake.LeftMotor.KP.getValue())
                    .outputRange(Intake.LeftMotor.MIN_POWER.getValue(), Intake.LeftMotor.MAX_POWER.getValue()).maxMotion
                    // Set MAXMotion parameters for position control
                    .maxVelocity(Intake.LeftMotor.MAX_VELOCITY.getValue())
                    .maxAcceleration(Intake.LeftMotor.MAX_ACCELERATION.getValue())
                    .allowedClosedLoopError(Intake.LeftMotor.MAX_CLOSED_LOOP_ERROR.getValue());
            leftMotor = new SparkFlex(Constants.CanId.Intake.LEFT_MOTOR, MotorType.kBrushless);
            leftMotor.configure(
                    intakeLeftConfig,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
            leftMotor.setInverted(true);
    
            // Right Motor
            intakeRightConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
            intakeRightConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Set PID values for position control
                    .p(Intake.RightMotor.KP.getValue())
                    .outputRange(Intake.RightMotor.MIN_POWER.getValue(), Intake.RightMotor.MAX_POWER.getValue()).maxMotion
                    // Set MAXMotion parameters for position control
                    .maxVelocity(Intake.RightMotor.MAX_VELOCITY.getValue())
                    .maxAcceleration(Intake.RightMotor.MAX_ACCELERATION.getValue())
                    .allowedClosedLoopError(Intake.RightMotor.MAX_CLOSED_LOOP_ERROR.getValue());
            rightMotor = new SparkFlex(Constants.CanId.Intake.RIGHT_MOTOR, MotorType.kBrushless);
            rightMotor.configure(
                    intakeLeftConfig,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
        }
    
        public void periodic() {
            sensingDistance = Intake.Nums.sensingDistance.getValue();
            motorSpeed = Intake.Nums.motorSpeed.getValue();
            
            SmartDashboard.putNumber("Intake Sensor", getSensorValue(intakeSensor));
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
    
        public int getSensorValue(LaserCan sensor) {
            Measurement measurement = sensor.getMeasurement();
            return measurement == null ? LASER_CAN_NO_MEASUREMENT : measurement.distance_mm;
    }

    public Command runMotors() {
        return runEnd(
                () -> {
                    leftMotor.set(motorSpeed);
                    rightMotor.set(motorSpeed);
                },
                () -> {
                    
                    leftMotor.stopMotor();
                    rightMotor.stopMotor();
                });
    }

    public Command runMotorsBack(){
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
        return runMotorsBack().until(() -> !coralInIntake());
    }

    public Command intakeCoral() {
        return runMotors().until(() -> coralInIntake());
    }
}
