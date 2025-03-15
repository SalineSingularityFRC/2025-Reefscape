package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import frc.robot.Constants;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
    private TalonFX mainMotor, algaeMotor;
    private LaserCan sensor;
    private double sensingDistance;
    private final VelocityTorqueCurrentFOC intakeSpeedRequest, outtakeSpeedRequest;
    private static final int LASER_CAN_NO_MEASUREMENT = -1;

    public AlgaeSubsystem() {
        mainMotor = new TalonFX(Constants.CanId.Algae.MAIN_MOTOR);
        algaeMotor = new TalonFX(Constants.CanId.Algae.ALGAE_MOTOR);
        sensor = new LaserCan(Constants.CanId.Algae.ALGAE_LASER);

        intakeSpeedRequest = new VelocityTorqueCurrentFOC(Constants.Algae.motorSpeedSlow.getValue()).withSlot(0);
        outtakeSpeedRequest = new VelocityTorqueCurrentFOC(Constants.Algae.motorSpeedFast.getValue()).withSlot(0);

        mainMotor.setPosition(0);
        mainMotor.setNeutralMode(NeutralModeValue.Brake);
        mainMotor.getConfigurator().apply(new TalonFXConfiguration());

        algaeMotor.setNeutralMode(NeutralModeValue.Brake);
        algaeMotor.getConfigurator().apply(new TalonFXConfiguration());
        
        TalonFXConfiguration mainConfig = new TalonFXConfiguration();
        mainConfig.Slot0.kP = Constants.Algae.kPMain.getValue();
        mainConfig.Slot0.kD = Constants.Algae.kDMain.getValue();
        mainMotor.getConfigurator().apply(mainConfig);

        TalonFXConfiguration algaeConfig = new TalonFXConfiguration();
        algaeConfig.Slot0.kP = Constants.Algae.kPAlgae.getValue();
        algaeConfig.Slot0.kD = Constants.Algae.kDAlgae.getValue();
        algaeMotor.getConfigurator().apply(algaeConfig);

        sensingDistance = Constants.Algae.sensingDistance.getValue();
    }

    public void periodic() {
        intakeSpeedRequest.Velocity = Constants.Algae.motorSpeedSlow.getValue();
        outtakeSpeedRequest.Velocity = Constants.Algae.motorSpeedFast.getValue();
    }

    public Command moveToPos(DoubleSupplier targetPos) {
        return new FunctionalCommand(
            () -> {
                mainMotor.setControl(new PositionDutyCycle(targetPos.getAsDouble()).withSlot(0).withEnableFOC(true));
            },
            () -> {

            },
            (_unused) -> {

            },
            () -> {
                return Math.abs(targetPos.getAsDouble() - mainMotor.getPosition().getValueAsDouble()) < Constants.Algae.MAX_CONTROL_ERROR_IN_COUNTS.getValue();
            },
            this);
    }

    public Command intakeAlgae() {
        return moveToPos(() -> Constants.Algae.INTAKE_POS.getValue()).alongWith(runAlgaeMotor().until(() -> algaeInSystem()));
    }

    public Command shootAlgae() {
        return moveToPos(() -> Constants.Algae.SHOOT_POS.getValue()).andThen(runAlgaeMotor().until(() -> !algaeInSystem()));
    }

    public Command runAlgaeMotor() {
        return runEnd(() -> {
            algaeMotor.setControl(algaeInSystem() ? outtakeSpeedRequest: intakeSpeedRequest); // If algae is in system, make motor speed fast, otherwise slow
        }, () -> {
            algaeMotor.stopMotor();
        });
    }

    public boolean algaeInSystem() {
        return sensor.getMeasurement() != null && getSensorValue(sensor) <= sensingDistance;
    }

    public int getSensorValue(LaserCan laserCan) {
        Measurement measurement = laserCan.getMeasurement();
        return measurement == null ? LASER_CAN_NO_MEASUREMENT: measurement.distance_mm;
    }

    public Command returnToHomePos() {
        return moveToPos(() -> 0).onlyIf(() -> !algaeInSystem());
    }
}
