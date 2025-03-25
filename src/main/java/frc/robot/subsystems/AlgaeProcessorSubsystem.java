package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;

public class AlgaeProcessorSubsystem extends SubsystemBase {
    private SparkFlex intakeMotor;
    private final VelocityTorqueCurrentFOC intakeRequest, spitRequest;
    private Servo ledController = new Servo(1);
    private Servo ledController2 = new Servo(2);
    private double dc;
    final double FREQ = 1000;
    final int maxPeriodInUS = (int) (1.0e6 / FREQ);
    final int minPeriodInUS = 0;
    final int middlePeriodInUS = (maxPeriodInUS - minPeriodInUS) / 2;
    public static final SparkFlexConfig motorConfig = new SparkFlexConfig();

    public AlgaeProcessorSubsystem() {
        intakeMotor = new SparkFlex(Constants.CanId.Processor.INTAKE_MOTOR, MotorType.kBrushless);
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


        intakeMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeRequest = new VelocityTorqueCurrentFOC(Constants.Processor.intakeSpeed.getValue()).withSlot(0);
        spitRequest = new VelocityTorqueCurrentFOC(Constants.Processor.spitSpeed.getValue()).withSlot(0);

        ledController.setPeriodMultiplier(PeriodMultiplier.k1X);
        ledController.setBoundsMicroseconds(maxPeriodInUS, 0, middlePeriodInUS, 0, minPeriodInUS);
        ledController.setBoundsMicroseconds(1950, 1504, 1500, 1496, 1050);
        ledController2.setPeriodMultiplier(PeriodMultiplier.k1X);
        ledController2.setBoundsMicroseconds(maxPeriodInUS, 0, middlePeriodInUS, 0, minPeriodInUS);
        ledController2.setBoundsMicroseconds(1950, 1504, 1500, 1496, 1050);
        dc = 1;

        // intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        // intakeMotor.getConfigurator().apply(new TalonFXConfiguration());

        // TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        // intakeConfig.Slot0.kP = Constants.Processor.kP.getValue();
        // intakeMotor.getConfigurator().apply(intakeConfig);
    }

    @Override
    public void periodic() {
        intakeRequest.Velocity = Constants.Processor.intakeSpeed.getValue();
        spitRequest.Velocity = Constants.Processor.spitSpeed.getValue();

        // dc = SmartDashboard.getNumber("Test/DutyCycle", dc);
        double pulseWidthInSeconds = this.dc * (1.0 / FREQ);
        int pulseWidthInMicroSeconds = (int) (pulseWidthInSeconds * 1e6);
        ledController.setPulseTimeMicroseconds(pulseWidthInMicroSeconds);
        ledController.set(dc);
        ledController2.setPulseTimeMicroseconds(pulseWidthInMicroSeconds);
        ledController2.set(dc);
        SmartDashboard.putNumber("Test/PulseWidthUS", pulseWidthInMicroSeconds);
        SmartDashboard.putNumber("Test/DutyCycle", dc);
    }

    public Command intakeProcessor() {
        return runEnd(() -> {
            intakeMotor.set(Constants.Processor.intakeSpeed.getValue());
        }, () -> {
            intakeMotor.stopMotor();
        });
    }

    public Command spitProcessor() {
        return runEnd(() -> {
            intakeMotor.set(Constants.Processor.spitSpeed.getValue());
        }, () -> {
            intakeMotor.stopMotor();
        });
    }

    public Command setDutyCycle(double dc) {
        return runOnce(() -> {
            this.dc = dc;
            // SmartDashboard.putNumber("Test/DutyCycle", dc);
        });
    }

    public void notCommandSetDutyCycle(double dc) {
        this.dc = dc;
    }
}