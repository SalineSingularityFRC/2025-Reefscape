package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {
    private SparkMax winchMotor;
    private double winchSpeed;
    public static final SparkMaxConfig intakeLeftConfig = new SparkMaxConfig();
    public static final SparkMaxConfig intakeRightConfig = new SparkMaxConfig();

    public ClimberSubsystem() {

        winchSpeed = Climber.WINCH_SPEED.getValue();

        // Winch Motor
        intakeRightConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
        winchMotor = new SparkMax(Constants.CanId.Intake.RIGHT_MOTOR, MotorType.kBrushless);
        winchMotor.configure(
                intakeLeftConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
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
