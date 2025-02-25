package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Trougth;

public class TrougthSubsystem extends SubsystemBase{
    private SparkFlex trougthMotor;
    private double trougthSpeed;
    private boolean isOverMax;
    private boolean isUnderMin;

    public TrougthSubsystem() {

        trougthSpeed = Trougth.TROUGTH_SPEED.getValue();

        // Trougth Motor

        trougthMotor = new SparkFlex(Constants.CanId.Trougth.TROUGTH_MOTOR, MotorType.kBrushless);
        trougthMotor.setInverted(true);
        /*
         * trougthMotor.configure(
         * intakeLeftConfig,
         * ResetMode.kResetSafeParameters,
         * PersistMode.kPersistParameters);
         */
    }

    public void periodic() {
        double actualMotorPos = trougthMotor.getEncoder().getPosition();
        isOverMax = actualMotorPos > Trougth.ENCODER_MAX_POS.getValue();
        isUnderMin = actualMotorPos < Trougth.ENCODER_MIN_POS.getValue();

        SmartDashboard.putNumber("Trougth/Trougth Position", actualMotorPos);
        SmartDashboard.putBoolean("Trougth/Over Max", isOverMax);
        SmartDashboard.putBoolean("Trougth/Under Min", isUnderMin);
    }

    public Command moveTrougthForward() {
        return runEnd(
                () -> {
                    if (!isUnderMin) {
                        trougthMotor.set(trougthSpeed);
                    } else {
                        trougthMotor.stopMotor();
                    }
                },
                () -> {
                    trougthMotor.stopMotor();
                });

    }

    public Command moveTrougthBack() {
        return runEnd(
                () -> {
                    if (!isOverMax) {
                        trougthMotor.set(-trougthSpeed);
                    } else {
                        trougthMotor.stopMotor();
                    }
                },
                () -> {
                    trougthMotor.stopMotor();
                });

    }

    // Runs the trougth to the intake position
    public Command moveToReady(){
        return moveTrougthForward().until(() -> isUnderMin);
    }

    // Runs the trougth up for climber
    public Command moveUp(){
        return moveTrougthBack().until(() -> isOverMax);
    }
}