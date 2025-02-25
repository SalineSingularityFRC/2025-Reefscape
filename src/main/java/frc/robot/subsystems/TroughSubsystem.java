package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Trough;

public class TroughSubsystem extends SubsystemBase {
    private SparkFlex troughMotor;
    private double troughSpeed;
    private boolean isOverMax;
    private boolean isUnderMin;
    private RelativeEncoder troughEncoder;
    public static final SparkFlexConfig troughMotorConfig = new SparkFlexConfig();
    private SparkClosedLoopController closedLoopController;

    public TroughSubsystem() {
        troughSpeed = Trough.TROUGH_SPEED.getValue();

        // Trougth Motor
        troughMotor = new SparkFlex(Constants.CanId.Trougth.TROUGH_MOTOR, MotorType.kBrushless);
        troughMotorConfig.idleMode(IdleMode.kBrake);
        troughMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control
                .p(Trough.KP.getValue())
                .d(Trough.KD.getValue())
                .outputRange(-1, 1).maxMotion
                // Set MAXMotion parameters for position control
                .maxVelocity(1000)
                .maxAcceleration(1000)
                .allowedClosedLoopError(0.1);
        troughMotor.configure(
                troughMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        troughEncoder = troughMotor.getEncoder();
        closedLoopController = troughMotor.getClosedLoopController();
    }

    public void periodic() {
        double actualMotorPos = troughEncoder.getPosition();
        troughSpeed = Trough.TROUGH_SPEED.getValue();
        isOverMax = actualMotorPos > Trough.ENCODER_MAX_POS.getValue();
        isUnderMin = actualMotorPos < Trough.ENCODER_MIN_POS.getValue();

        SmartDashboard.putNumber("Trough/Position", actualMotorPos);
        SmartDashboard.putBoolean("Trough/Over Max", isOverMax);
        SmartDashboard.putBoolean("Trough/Under Min", isUnderMin);
    }

    public Command moveTroughForward() {
        return runEnd(
                () -> {
                    if (!isOverMax) {
                        troughMotor.set(troughSpeed);
                    } else {
                        troughMotor.stopMotor();
                    }
                },
                () -> {
                    troughMotor.stopMotor();
                });
    }

    public Command moveTroughBack() {
        return runEnd(
                () -> {
                    if (!isUnderMin) {
                        troughMotor.set(-troughSpeed);
                    } else {
                        troughMotor.stopMotor();
                    }
                },
                () -> {
                    troughMotor.stopMotor();
                });
    }

    public Command goToPosition(double position) {
        return new InstantCommand(() -> {
            closedLoopController.setReference(position, ControlType.kMAXMotionPositionControl);
        });
    }

    public Command moveToZeroPostion() {
        return goToPosition(Trough.HOME_POSITION.getValue());
    }

    public Command moveToClimbPositon() {
        return goToPosition(Trough.CLIMB_POSITION.getValue());
    }
}