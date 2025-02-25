package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {
    private TalonFX winchMotor;
    private double winchSpeed;
    private boolean isOverMax;
    private boolean isUnderMin;

    public ClimberSubsystem() {

        winchSpeed = Climber.WINCH_SPEED.getValue();

        // Winch Motor

        winchMotor = new TalonFX(Constants.CanId.Climber.MOTOR);
        winchMotor.setPosition(0);
        winchMotor.setInverted(true);
        /*
         * winchMotor.configure(
         * intakeLeftConfig,
         * ResetMode.kResetSafeParameters,
         * PersistMode.kPersistParameters);
         */
    }

    public void periodic() {
        StatusSignal<Angle> currentMotorPos = winchMotor.getPosition();
        double actualMotorPos = currentMotorPos.getValueAsDouble();
        isOverMax = actualMotorPos > Climber.ENCODER_MAX_POS.getValue();
        isUnderMin = actualMotorPos < Climber.ENCODER_MIN_POS.getValue();

        SmartDashboard.putNumber("Climber/CLimber Position", actualMotorPos);
        SmartDashboard.putBoolean("Climber/Over Max", isOverMax);
        SmartDashboard.putBoolean("Climber/Under Min", isUnderMin);
    }

    public Command moveWinchForward() {
        return runEnd(
                () -> {
                    if (!isOverMax) {
                        winchMotor.set(winchSpeed);
                    } else {
                        winchMotor.stopMotor();
                    }
                },
                () -> {
                    winchMotor.stopMotor();
                });

    }

    public Command moveWinchBack() {
        return runEnd(
                () -> {
                    if (!isUnderMin) {
                        winchMotor.set(-winchSpeed);
                    } else {
                        winchMotor.stopMotor();
                    }
                },
                () -> {
                    winchMotor.stopMotor();
                });

    }
}
