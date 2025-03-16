package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeProcessorSubsystem extends SubsystemBase {
    private TalonFX intakeMotor;
    private final VelocityTorqueCurrentFOC intakeRequest, spitRequest;

    public AlgaeProcessorSubsystem() {
        intakeMotor = new TalonFX(Constants.CanId.Processor.INTAKE_MOTOR);
        intakeRequest = new VelocityTorqueCurrentFOC(Constants.Processor.intakeSpeed.getValue()).withSlot(0);
        spitRequest = new VelocityTorqueCurrentFOC(Constants.Processor.spitSpeed.getValue()).withSlot(0);

        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.Slot0.kP = Constants.Processor.kP.getValue();
        intakeMotor.getConfigurator().apply(intakeConfig);
    }

    public void periodic() {
        intakeRequest.Velocity = Constants.Processor.intakeSpeed.getValue();
        spitRequest.Velocity = Constants.Processor.spitSpeed.getValue();
    }

    public Command intakeProcessor() {
        return runEnd(() -> {
            intakeMotor.setControl(intakeRequest);
        }, () -> {
            intakeMotor.stopMotor();
        });
    }

    public Command spitProcessor() {
        return runEnd(() -> {
            intakeMotor.setControl(spitRequest);
        }, () -> {
            intakeMotor.stopMotor();
        });
    }
}
