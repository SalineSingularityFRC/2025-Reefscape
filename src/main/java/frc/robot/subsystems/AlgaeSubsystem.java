package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
    private TalonFX mainMotor, algaeMotor;

    public AlgaeSubsystem() {
        mainMotor = new TalonFX(Constants.CanId.Algae.MAIN_MOTOR);
        algaeMotor = new TalonFX(Constants.CanId.Algae.ALGAE_MOTOR);

        mainMotor.setPosition(0);
        mainMotor.setNeutralMode(NeutralModeValue.Brake);
        mainMotor.getConfigurator().apply(new TalonFXConfiguration());

        algaeMotor.setNeutralMode(NeutralModeValue.Brake);
        algaeMotor.getConfigurator().apply(new TalonFXConfiguration());
        
        TalonFXConfiguration mainConfig = new TalonFXConfiguration();
        mainConfig.Slot0.kP = Constants.Algae.kP.getValue();
        mainConfig.Slot0.kD = Constants.Algae.kD.getValue();
        mainMotor.getConfigurator().apply(mainConfig);
    }

    public Command moveToPos(double targetRotations) {
        return new FunctionalCommand(() -> {},
        () -> {
            mainMotor.setControl(new MotionMagicVoltage(targetRotations).withSlot(0).withEnableFOC(true));
        },
        (_unused) -> {},
        () -> (), this);
    }
}
