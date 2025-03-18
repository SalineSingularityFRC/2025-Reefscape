package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AlgaeSubsystem extends SubsystemBase {
    private TalonFX mainMotor, algaeMotor;
    private LaserCan sensor;
    private double sensingDistance;
    private final VelocityTorqueCurrentFOC intakeSpeedRequest, outtakeSpeedRequest;
    private final PositionTorqueCurrentFOC algaeMotorHoldRequest;
    private final PositionDutyCycle mainMotorHoldRequest;
    private static final int LASER_CAN_NO_MEASUREMENT = -1;
    private final Timer timer;
    private boolean hasAlgae;
    private double targetPosition;

    public AlgaeSubsystem() {
        mainMotor = new TalonFX(Constants.CanId.Algae.MAIN_MOTOR);
        algaeMotor = new TalonFX(Constants.CanId.Algae.ALGAE_MOTOR);
        sensor = new LaserCan(Constants.CanId.Algae.ALGAE_LASER);

        intakeSpeedRequest = new VelocityTorqueCurrentFOC(Constants.Algae.motorSpeedSlow.getValue()).withSlot(0);
        outtakeSpeedRequest = new VelocityTorqueCurrentFOC(Constants.Algae.motorSpeedFast.getValue()).withSlot(0);
        algaeMotorHoldRequest = new PositionTorqueCurrentFOC(algaeMotor.getPosition().getValueAsDouble()).withSlot(1);
        mainMotorHoldRequest = new PositionDutyCycle(mainMotor.getPosition().getValueAsDouble()).withSlot(0);

        mainMotor.setPosition(0);

        mainMotor.setNeutralMode(NeutralModeValue.Brake);
        mainMotor.getConfigurator().apply(new TalonFXConfiguration());

        algaeMotor.setNeutralMode(NeutralModeValue.Brake);
        algaeMotor.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration mainConfig = new TalonFXConfiguration();
        mainConfig.Slot0.kP = Constants.Algae.kPMain.getValue();
        mainConfig.Slot0.kD = Constants.Algae.kDMain.getValue();
        mainConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        mainConfig.CurrentLimits.SupplyCurrentLimit = 20;
        mainMotor.getConfigurator().apply(mainConfig);

        TalonFXConfiguration algaeConfig = new TalonFXConfiguration();
        algaeConfig.Slot0.kP = Constants.Algae.kPAlgae.getValue();
        algaeConfig.Slot0.kD = Constants.Algae.kDAlgae.getValue();
        algaeConfig.Slot1.kP = Constants.Algae.kP1Algae.getValue();
        algaeConfig.Slot1.kD = Constants.Algae.kD1Algae.getValue();
        algaeMotor.getConfigurator().apply(algaeConfig);

        sensingDistance = Constants.Algae.sensingDistance.getValue();

        timer = new Timer();
        hasAlgae = false;

        setDefaultCommand(holdCommand());
    }

    public void periodic() {
        // if (canSeeAlgae() && timer.hasElapsed(2)) {
        // hasAlgae = true;
        // }

        intakeSpeedRequest.Velocity = Constants.Algae.motorSpeedSlow.getValue();
        outtakeSpeedRequest.Velocity = Constants.Algae.motorSpeedFast.getValue();

        // SmartDashboard.putNumber("AlgaeHinge/TargetPosition", targetPosition);
        // SmartDashboard.putNumber("AlgaeHinge/ActualPosition", mainMotor.getPosition().getValueAsDouble());
    }

    public Command holdCommand() {
        return runOnce(() -> {
            algaeMotor.setControl(algaeMotorHoldRequest.withSlot(1));
            mainMotor.setControl(mainMotorHoldRequest.withSlot(0));
        });
    }

    public Command moveToPos(DoubleSupplier targetPos) {
        return new FunctionalCommand(
                () -> {
                    targetPosition = targetPos.getAsDouble();
                    mainMotor
                            .setControl(new PositionDutyCycle(targetPos.getAsDouble()).withSlot(0).withEnableFOC(true));
                },
                () -> {

                },
                (_unused) -> {

                },
                () -> {
                    return Math.abs(targetPos.getAsDouble()
                            - mainMotor.getPosition().getValueAsDouble()) < Constants.Algae.MAX_CONTROL_ERROR_IN_COUNTS
                                    .getValue();
                },
                this);
    }

    public Command moveToIntakePos() {
        return moveToPos(() -> Constants.Algae.INTAKE_POS.getValue());
    }

    public Command shootAlgae() {
        return moveToPos(() -> Constants.Algae.SHOOT_POS.getValue())
                .andThen(spitAlgaeMotor().until(() -> !canSeeAlgae()));
    }

    public Command intake() {
        return intakeAlgaeMotor().until(() -> canSeeAlgae()).andThen(new WaitCommand(0.05)).andThen(hold(3));
    }

    public Command intakeAlgaeMotor() {
        return run(() -> {
            algaeMotor.setControl(intakeSpeedRequest); // If algae is in system, make motor speed fast, otherwise slow
        });
    }

    public Command spitAlgaeMotor() {
        return run(() -> {
            algaeMotor.setControl(outtakeSpeedRequest);
        }).andThen(hold(0));
    }

    public Command hold(double extra) {
        return runOnce(() -> {
            algaeMotorHoldRequest.Position = algaeMotor.getPosition().getValueAsDouble() + extra;
        });
    }

    // public Command stopAlgaeMotor() {
    // return runOnce(() -> {
    // algaeMotor.stopMotor();
    // });
    // }

    public boolean canSeeAlgae() {
        return sensor.getMeasurement() != null && getSensorValue(sensor) <= sensingDistance;
    }

    public int getSensorValue(LaserCan laserCan) {
        Measurement measurement = laserCan.getMeasurement();
        return measurement == null ? LASER_CAN_NO_MEASUREMENT : measurement.distance_mm;
    }

    public Command returnToHomePos() {
        return moveToPos(() -> 0).onlyIf(() -> !canSeeAlgae());
    }

    public Command mainMotorHoldCommand() {
        return runOnce(() -> {
            mainMotorHoldRequest.Position = mainMotor.getPosition().getValueAsDouble();
        });
    }

    public Command manualControlForward() {
        return runEnd(() -> {
            mainMotor.setControl(new DutyCycleOut(Constants.Algae.manualSpeed.getValue()));
        }, () -> {
            // mainMotor.setControl(new DutyCycleOut(0));
            mainMotorHoldCommand();
        });
    }

    public Command manualControlBackwards() {
        return runEnd(() -> {
            mainMotor.setControl(new DutyCycleOut(-Constants.Algae.manualSpeed.getValue()));
        }, () -> {
            // mainMotor.setControl(new DutyCycleOut(0));
            mainMotorHoldCommand();
        });
    }
}
