package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.Constants.Drive;

public class CameraDriveToPose extends Command {
    private final SwerveSubsystem m_swerve;
    private final Supplier<Pose3d> targetPose, currentPose;
    private PIDController rotationController;
    private PIDController xDriveController;
    private PIDController yDriveController;
    private SimpleMotorFeedforward rotationFeedForward = new SimpleMotorFeedforward(0, 0);

    public CameraDriveToPose(SwerveSubsystem swerve, Supplier<Pose3d> currentPoseSupplier, Supplier<Pose3d> targetPoseSupplier,
            Matrix<N3, N1> normalVector, double targetAngle) {
        this.currentPose = targetPoseSupplier;
        this.targetPose = currentPoseSupplier;
        m_swerve = swerve;
        addRequirements(swerve);

        rotationController = new PIDController(
                Drive.PID_DRIVE_ROTATION_KP.getValue(),
                Drive.PID_DRIVE_ROTATION_KI.getValue(),
                Drive.PID_DRIVE_ROTATION_KD.getValue());

        rotationController.setSetpoint(0);
        rotationController.setTolerance(Drive.PID_DRIVE_ROTATION_TOLERANCE.getValue());

        xDriveController = new PIDController(
                Drive.PID_DRIVE_X_KP.getValue(),
                Drive.PID_DRIVE_X_KI.getValue(),
                Drive.PID_DRIVE_X_KD.getValue());

        xDriveController.setSetpoint(0);
        xDriveController.setTolerance(Drive.PID_DRIVE_X_TOLERANCE.getValue());

        yDriveController = new PIDController(
                Drive.PID_DRIVE_Y_KP.getValue(),
                Drive.PID_DRIVE_Y_KI.getValue(),
                Drive.PID_DRIVE_Y_KD.getValue());

        yDriveController.setSetpoint(0);
        yDriveController.setTolerance(Drive.PID_DRIVE_Y_TOLERANCE.getValue());
    }

    public void execute() {
        if (Drive.PID_DRIVE_TUNING.getValue() > 0) {
            rotationController.setTolerance(Drive.PID_DRIVE_ROTATION_TOLERANCE.getValue());
            rotationController.setPID(
                Drive.PID_DRIVE_ROTATION_KP.getValue(),
                Drive.PID_DRIVE_ROTATION_KI.getValue(),
                Drive.PID_DRIVE_ROTATION_KD.getValue());
            xDriveController.setTolerance(Drive.PID_DRIVE_X_TOLERANCE.getValue());
            xDriveController.setPID(
                Drive.PID_DRIVE_X_KP.getValue(),
                Drive.PID_DRIVE_X_KI.getValue(),
                Drive.PID_DRIVE_X_KD.getValue());
            yDriveController.setTolerance(Drive.PID_DRIVE_Y_TOLERANCE.getValue());
            yDriveController.setPID(
                Drive.PID_DRIVE_Y_KP.getValue(),
                Drive.PID_DRIVE_Y_KI.getValue(),
                Drive.PID_DRIVE_Y_KD.getValue());
        }

        Pose3d currentPose = this.currentPose.get();
        Pose3d targetPose = this.targetPose.get();

        double dr = rotationController.calculate(currentPose.getRotation().getAngle(), targetPose.getRotation().getAngle());
        double dx = xDriveController.calculate(currentPose.getTranslation().getX(), targetPose.getTranslation().getX());
        double dy = yDriveController.calculate(currentPose.getTranslation().getY(), targetPose.getTranslation().getY());

        dr = MathUtil.clamp(dr, -Drive.PID_DRIVE_MAX_ROTATION_SPEED.getValue(), Drive.PID_DRIVE_MAX_ROTATION_SPEED.getValue());
        dx = MathUtil.clamp(dx, -Drive.PID_DRIVE_MAX_DRIVE_X_SPEED.getValue(), Drive.PID_DRIVE_MAX_DRIVE_X_SPEED.getValue()); 
        dy = MathUtil.clamp(dy, -Drive.PID_DRIVE_MAX_DRIVE_Y_SPEED.getValue(), Drive.PID_DRIVE_MAX_DRIVE_Y_SPEED.getValue()); 

        m_swerve.drive(dr, dx, dy, false, 1.0);
    }

    public boolean isFinished() {
        return rotationController.atSetpoint() && xDriveController.atSetpoint() && yDriveController.atSetpoint();
    }
}
