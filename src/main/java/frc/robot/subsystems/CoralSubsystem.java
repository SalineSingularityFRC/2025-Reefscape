package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;

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

public class CoralSubsystem extends SubsystemBase {
    private TalonFX rightMotor;
    private TalonFX leftMotor;
    private LaserCan troughSensor;
    private LaserCan intakeSensor;
    private LaserCan shooterSensor;
    private double intakeDistance, shooterDistance;
    private double troughSenserDistance;
    private final VelocityTorqueCurrentFOC slowVelocityRequest, fastVelocityRequest;
    private final VelocityTorqueCurrentFOC shooterVelocityRequest;
    private SparkFlex conveyorMotor;
    private SparkClosedLoopController conveyorClosedLoopController;
    private boolean laserCanLogicOverride = false;
    public static final SparkFlexConfig motorConfig = new SparkFlexConfig();

    private static final int LASER_CAN_NO_MEASUREMENT = -1;

    public CoralSubsystem() {
        troughSensor = new LaserCan(Constants.CanId.Intake.TROUGH_LASER);
        intakeSensor = new LaserCan(Constants.CanId.Intake.INTAKE_LASER);
        shooterSensor = new LaserCan(Constants.CanId.Intake.SHOOTER_LASER);

        conveyorMotor = new SparkFlex(Constants.CanId.Processor.INTAKE_MOTOR, MotorType.kBrushless);
        motorConfig.idleMode(IdleMode.kBrake)
                // .smartCurrentLimit(Constants.Processor.MotorStuff.MAX_CURRENT_IN_A.getValue());
                .voltageCompensation(Constants.Processor.MotorStuff.VOLTAGE_COMPENSATION_IN_V.getValue());
        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control
                .pidf(Constants.Processor.kP.getValue(), 0,
                        0, 0, ClosedLoopSlot.kSlot0)
                .outputRange(Constants.Processor.MotorStuff.MIN_POWER.getValue(),
                        Constants.Processor.MotorStuff.MAX_POWER.getValue()).maxMotion
                // Set MAXMotion parameters for position control
                .maxVelocity(Constants.Processor.MotorStuff.MAX_VELOCITY_RPM.getValue())
                .maxAcceleration(Constants.Processor.MotorStuff.MAX_ACCEL_RPM_PER_S.getValue())
                .allowedClosedLoopError(Constants.Processor.MotorStuff.MAX_CONTROL_ERROR_IN_COUNTS.getValue());
        
        conveyorClosedLoopController = conveyorMotor.getClosedLoopController();

        conveyorMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeDistance = Intake.Nums.intakeDistance.getValue();
        shooterDistance = Intake.Nums.shooterDistance.getValue();
        troughSenserDistance = Intake.Nums.troughSenserDistance.getValue();
        laserCanLogicOverride = Intake.Nums.laserCanLogicOverride.getValue();

        slowVelocityRequest = new VelocityTorqueCurrentFOC(Intake.Nums.motorSpeedSlow.getValue()).withSlot(0); // 30
        fastVelocityRequest = new VelocityTorqueCurrentFOC(Intake.Nums.motorSpeed.getValue()).withSlot(0); // 70
        shooterVelocityRequest = new VelocityTorqueCurrentFOC(Intake.Nums.shooterSpeed.getValue()).withSlot(0); // 70

        // Left Motor
        // intakeLeftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
        // intakeLeftConfig.closedLoop
        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // // Set PID values for position control
        // .p(Intake.LeftMotor.KP.getValue())
        // .outputRange(Intake.LeftMotor.MIN_POWER.getValue(),
        // Intake.LeftMotor.MAX_POWER.getValue()).maxMotion
        // // Set MAXMotion parameters for position control
        // .maxVelocity(Intake.LeftMotor.MAX_VELOCITY.getValue())
        // .maxAcceleration(Intake.LeftMotor.MAX_ACCELERATION.getValue())
        // .allowedClosedLoopError(Intake.LeftMotor.MAX_CLOSED_LOOP_ERROR.getValue());
        leftMotor = new TalonFX(Constants.CanId.Intake.LEFT_MOTOR);
        leftMotor.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration talonLeftConfig = new TalonFXConfiguration();
        talonLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Brake Mode
        talonLeftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonLeftConfig.CurrentLimits.SupplyCurrentLimit = 40; // Max Current
        talonLeftConfig.Voltage.PeakForwardVoltage = 12;
        talonLeftConfig.Voltage.PeakReverseVoltage = -12;
        talonLeftConfig.Slot0.kP = Intake.LeftMotor.KP.getValue();
        talonLeftConfig.Slot0.kS = Intake.LeftMotor.KS.getValue();
        talonLeftConfig.Slot0.kV = Intake.LeftMotor.KV.getValue();
        // talonLeftConfig.MotorOutput.PeakReverseDutyCycle =
        // Intake.LeftMotor.MIN_POWER.getValue();
        // talonLeftConfig.MotorOutput.PeakForwardDutyCycle =
        // Intake.LeftMotor.MAX_POWER.getValue();
        // talonLeftConfig.MotionMagic.MotionMagicCruiseVelocity =
        // Intake.LeftMotor.MAX_VELOCITY.getValue();
        // talonLeftConfig.MotionMagic.MotionMagicAcceleration =
        // Intake.LeftMotor.MAX_ACCELERATION.getValue();
        leftMotor.getConfigurator().apply(talonLeftConfig, 0.05);

        // leftMotor.configure(
        // intakeLeftConfig,
        // ResetMode.kResetSafeParameters,
        // PersistMode.kPersistParameters);
        leftMotor.setInverted(true);

        // Right Motor
        // intakeRightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
        // intakeRightConfig.closedLoop
        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // // Set PID values for position control
        // .p(Intake.RightMotor.KP.getValue())
        // .outputRange(Intake.RightMotor.MIN_POWER.getValue(),
        // Intake.RightMotor.MAX_POWER.getValue()).maxMotion
        // // Set MAXMotion parameters for position control
        // .maxVelocity(Intake.RightMotor.MAX_VELOCITY.getValue())
        // .maxAcceleration(Intake.RightMotor.MAX_ACCELERATION.getValue())
        // .allowedClosedLoopError(Intake.RightMotor.MAX_CLOSED_LOOP_ERROR.getValue());
        rightMotor = new TalonFX(Constants.CanId.Intake.RIGHT_MOTOR);
        rightMotor.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration talonRightConfig = new TalonFXConfiguration();
        talonRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Brake Mode
        talonRightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonRightConfig.CurrentLimits.SupplyCurrentLimit = 40; // Max Current
        talonRightConfig.Voltage.PeakForwardVoltage = 12;
        talonRightConfig.Voltage.PeakReverseVoltage = -12;
        talonRightConfig.Slot0.kP = Intake.RightMotor.KP.getValue();
        talonRightConfig.Slot0.kS = Intake.RightMotor.KS.getValue();
        talonRightConfig.Slot0.kV = Intake.RightMotor.KV.getValue();
        // talonRightConfig.MotorOutput.PeakReverseDutyCycle =
        // Intake.RightMotor.MIN_POWER.getValue();
        // talonRightConfig.MotorOutput.PeakForwardDutyCycle =
        // Intake.RightMotor.MAX_POWER.getValue();
        // talonRightConfig.MotionMagic.MotionMagicCruiseVelocity =
        // Intake.RightMotor.MAX_VELOCITY.getValue();
        // talonRightConfig.MotionMagic.MotionMagicAcceleration =
        // Intake.RightMotor.MAX_ACCELERATION.getValue();
        rightMotor.getConfigurator().apply(talonRightConfig);
        SignalLogger.stop();
    }

    public void periodic() {
        intakeDistance = Intake.Nums.intakeDistance.getValue();
        troughSenserDistance = Intake.Nums.troughSenserDistance.getValue();
        laserCanLogicOverride = Intake.Nums.laserCanLogicOverride.getValue();

        slowVelocityRequest.Velocity = Intake.Nums.motorSpeedSlow.getValue();
        fastVelocityRequest.Velocity = Intake.Nums.motorSpeed.getValue();
        shooterVelocityRequest.Velocity = Intake.Nums.shooterSpeed.getValue();

        // SmartDashboard.putNumber("Intake Sensor", getSensorValue(intakeSensor));
        // SmartDashboard.putNumber("Shooter Sensor", getSensorValue(shooterSensor));
        SmartDashboard.putBoolean("Coral in intake", coralInIntake());
        SmartDashboard.putBoolean("Coral in shooter", coralInShooter());
        SmartDashboard.putBoolean("Coral In Trough", coralInTrough());
        // SmartDashboard.putBoolean("Ready Shoot", readyToShoot());
        // SmartDashboard.putBoolean("NoCoralDetected", noCoralDetected());
        // SmartDashboard.putBoolean("IntakeSensorFunctional",
        // intakeSensorIsFunctional());
        // SmartDashboard.putBoolean("elevator can move",
        // elevator_can_move.getAsBoolean());
        // SmartDashboard.putNumber("Intake/Current Speed",
        // Math.abs(leftMotor.getVelocity().getValueAsDouble()));
        // SmartDashboard.putNumber("Intake/Intake Sensor Distance",
        // intakeSensor.getMeasurement().distance_mm);
        // SmartDashboard.putNumber("Intake/Shooter Sensor Distance",
        // shooterSensor.getMeasurement().distance_mm);
    }

    public boolean intakeCanSeeCoral(LaserCan sensor) {
        // if (!intakeSensorIsFunctional()) {
        // return false;
        // }

        // if (sensor.getMeasurement() != null && getSensorValue(sensor) <=
        // intakeDistance) {
        // return true;
        // }

        // return false;

        int sensorValue = getSensorValue(sensor);
        SmartDashboard.putNumber("Intake/Intake Sensor Distance", sensorValue);
        return sensorValue <= intakeDistance && sensorValue != LASER_CAN_NO_MEASUREMENT;
    }

    public boolean shooterCanSeeCoral(LaserCan sensor) {
        // if (!intakeSensorIsFunctional()) {
        // return false;
        // }

        // if (sensor.getMeasurement() != null && getSensorValue(sensor) <=
        // shooterDistance) {
        // return true;
        // }

        // return false;

        int sensorValue = getSensorValue(sensor);
        return sensorValue <= intakeDistance && sensorValue != LASER_CAN_NO_MEASUREMENT;
    }

    public boolean coralInIntake() {
        return intakeCanSeeCoral(intakeSensor);
    }

    public boolean coralInShooter() {
        return shooterCanSeeCoral(shooterSensor);
    }

    public boolean coralInTrough() {
        int sensorValue = getSensorValue(troughSensor);
        return sensorValue <= troughSenserDistance && sensorValue != LASER_CAN_NO_MEASUREMENT;
        // return troughSensor.getMeasurement() != null && getSensorValue(troughSensor)
        // <= troughSenserDistance;
    }

    public boolean readyToShoot() {
        return shooterCanSeeCoral(shooterSensor) && !intakeCanSeeCoral(intakeSensor);
    }

    public boolean noCoralDetected() {
        return !coralInShooter() && !coralInIntake();
    }

    BooleanSupplier elevator_can_move = () -> {
        return laserCanLogicOverride || ((readyToShoot() || noCoralDetected()) && intakeSensorIsFunctional());
    };

    public boolean intakeSensorIsFunctional() {
        return getSensorValue(shooterSensor) != LASER_CAN_NO_MEASUREMENT;
    }

    public int getSensorValue(LaserCan sensor) {
        Measurement measurement = sensor.getMeasurement();
        return measurement == null ? LASER_CAN_NO_MEASUREMENT : measurement.distance_mm;
    }

    // This Command is for driving away from the coral station when the coral enters
    // the trough
    public Command waitUntilCoral() {
        return run(() -> {

        }).until(() -> coralInTrough() || coralInIntake());
    }

    public boolean isMotorRunning() {
        return Math.abs(rightMotor.get()) > 0.01;
    }

    // public Command runMotorsBack() {
    // return runEnd(
    // () -> {
    // leftIntakeClosedLoopController.setReference(-motorSpeed,
    // ControlType.kVelocity);
    // rightIntakeClosedLoopController.setReference(-motorSpeed,
    // ControlType.kVelocity);
    // // leftMotor.set(-motorSpeed);
    // // rightMotor.set(-motorSpeed);
    // },
    // () -> {
    // leftMotor.stopMotor();
    // rightMotor.stopMotor();
    // });
    // }

    public Command shootCoral() {
        return runEnd(
                () -> {
                    leftMotor.setControl(shooterVelocityRequest);
                    rightMotor.setControl(shooterVelocityRequest);
                },
                () -> {
                    leftMotor.stopMotor();
                    rightMotor.stopMotor();
                }).until(() -> (!coralInShooter() && !coralInIntake()) && !laserCanLogicOverride);
    }

    public Command shootL1Coral() {
        return runEnd(
                () -> {
                    leftMotor.setControl(shooterVelocityRequest);
                    rightMotor.setControl(slowVelocityRequest);
                },
                () -> {
                    leftMotor.stopMotor();
                    rightMotor.stopMotor();
                }).until(() -> (!coralInShooter() && !coralInIntake()) && !laserCanLogicOverride);
    }

    // ONLY ADD SMARTDASHBOARD FOR DEBUGGGING, causes delay in CAN first time you
    // run the command
    public Command intakeCoral() {
        return runEnd(
                () -> {
                    leftMotor.setControl(
                            coralInShooter() || laserCanLogicOverride ? slowVelocityRequest : fastVelocityRequest);
                    rightMotor.setControl(
                            coralInShooter() || laserCanLogicOverride ? slowVelocityRequest : fastVelocityRequest);
                    conveyorMotor.set(Intake.Nums.conveyorSpeed.getValue());
                    // conveyorClosedLoopController.setReference(Intake.Nums.conveyorSpeed.getValue(), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
                },
                () -> {
                    conveyorMotor.stopMotor();
                    leftMotor.stopMotor();
                    rightMotor.stopMotor();
                    // SmartDashboard.putNumber("Intake/Target Speed", 0);
                }).until(() -> coralInShooter() && !coralInIntake());

    }

    public Command conveyorReverse() {
        return runEnd(
                () -> {
                    conveyorMotor.set(-1 * Intake.Nums.conveyorSpeed.getValue());
                    // conveyorClosedLoopController.setReference(-Intake.Nums.conveyorSpeed.getValue(), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
                },
                () -> {
                    // conveyorMotor.set(0);
                    conveyorMotor.stopMotor();
                });
    }
}
