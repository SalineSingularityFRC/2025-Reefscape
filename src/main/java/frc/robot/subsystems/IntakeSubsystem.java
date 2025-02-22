package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase {
    private SparkFlex rightMotor;
    private SparkFlex leftMotor;
    private LaserCan intakeSensor;
    private LaserCan shooterSensor;
    private double motorSpeed;
    private double sensingDistance;
    private SparkClosedLoopController leftIntakeClosedLoopController;
    private SparkClosedLoopController rightIntakeClosedLoopController;
        private Double motorSpeedSlow;
        public static final SparkFlexConfig intakeLeftConfig = new SparkFlexConfig();
        public static final SparkFlexConfig intakeRightConfig = new SparkFlexConfig();
    
        private static final int LASER_CAN_NO_MEASUREMENT = -1;
    
        public IntakeSubsystem() {
            intakeSensor = new LaserCan(Constants.CanId.Intake.INTAKE_LASER);
            shooterSensor = new LaserCan(Constants.CanId.Intake.SHOOTER_LASER);
    
            sensingDistance = Intake.Nums.sensingDistance.getValue();
            motorSpeed = Intake.Nums.motorSpeed.getValue();
            motorSpeedSlow = Intake.Nums.motorSpeedSlow.getValue();

            // Left Motor
            intakeLeftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
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
            intakeRightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
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
    
            leftIntakeClosedLoopController = leftMotor.getClosedLoopController();
            rightIntakeClosedLoopController = rightMotor.getClosedLoopController();
        }
    
        public void periodic() {
            sensingDistance = Intake.Nums.sensingDistance.getValue();
            motorSpeed = Intake.Nums.motorSpeed.getValue();
            motorSpeedSlow = Intake.Nums.motorSpeedSlow.getValue();

        SmartDashboard.putNumber("Intake Sensor", getSensorValue(intakeSensor));
        SmartDashboard.putNumber("Shooter Sensor", getSensorValue(shooterSensor));
        SmartDashboard.putBoolean("Coral in intake", coralInIntake());
        SmartDashboard.putBoolean("Coral in shooter", coralInShooter());
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

    public boolean coralInShooter(){
        return canSeeCoral(shooterSensor);
    }

    public boolean readyToShoot(LaserCan intakeSensor, LaserCan shooterSensor) {
        return canSeeCoral(shooterSensor) && !canSeeCoral(intakeSensor);
    }

    public boolean noCoralDetected() {
        return !coralInShooter() && !coralInIntake();
    }

    BooleanSupplier supplier_elevatar_move = () -> {
        return readyToShoot(intakeSensor, shooterSensor) || noCoralDetected();
    };

    public int getSensorValue(LaserCan sensor) {
        Measurement measurement = sensor.getMeasurement();
        return measurement == null ? LASER_CAN_NO_MEASUREMENT : measurement.distance_mm;
    }

    public Command runMotors() {
        return runEnd(
                () -> {
                    double speed = coralInShooter() ? motorSpeedSlow : motorSpeed;
                    // leftIntakeClosedLoopController.setReference(speed, ControlType.kVelocity);
                    // rightIntakeClosedLoopController.setReference(speed, ControlType.kVelocity);
                    leftMotor.set(speed);
                    rightMotor.set(speed);
                },
                () -> {
                    leftMotor.stopMotor();
                    rightMotor.stopMotor();
                });
    }

    // public Command runMotorsBack() {
    //     return runEnd(
    //             () -> {
    //                 leftIntakeClosedLoopController.setReference(-motorSpeed, ControlType.kVelocity);
    //                 rightIntakeClosedLoopController.setReference(-motorSpeed, ControlType.kVelocity);
    //                 // leftMotor.set(-motorSpeed);
    //                 // rightMotor.set(-motorSpeed);
    //             },
    //             () -> {
    //                 leftMotor.stopMotor();
    //                 rightMotor.stopMotor();
    //             });
    // }

    public Command shootCoral() {
        return runMotors().until(() -> !coralInShooter() && !coralInIntake());
    }

    public Command intakeCoral() {
        return runMotors().until(() -> coralInShooter() && !coralInIntake());
    }
}
