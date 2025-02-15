package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {
    private TalonFX winchMotor;
    private double winchSpeed;
   

    public ClimberSubsystem() {

        winchSpeed = Climber.WINCH_SPEED.getValue();

        // Winch Motor

        winchMotor = new TalonFX(Constants.CanId.Intake.RIGHT_MOTOR);
        /*winchMotor.configure(
                intakeLeftConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);*/
    }

    public void periodic() {

    }

    public Command moveWinchForward() {
        return runEnd(
                () -> {
                    winchMotor.set(-winchSpeed);
                },
                () -> {
                    winchMotor.stopMotor();
                });

    }

    public Command moveWinchBack() {
        return runEnd(
                () -> {
                    winchMotor.set(winchSpeed);
                },
                () -> {
                    winchMotor.stopMotor();
                });

    }
}
