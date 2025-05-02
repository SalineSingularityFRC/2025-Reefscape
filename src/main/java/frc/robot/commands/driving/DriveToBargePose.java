package frc.robot.commands.driving;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.Constants.Drive;

/**
 * A command that takes in a field relative pose and drives the robot to it via PID
 */
public class DriveToBargePose extends Command {
    private final SwerveSubsystem m_swerve;
    private final Supplier<Pose2d> targetPose, currentPose;
    private PIDController rotationController;
    private PIDController xDriveController;
    private PIDController yDriveController;

    public DriveToBargePose(SwerveSubsystem swerve, Supplier<Pose2d> currentPoseSupplier, Supplier<Pose2d> targetPoseSupplier) {
        this.currentPose = currentPoseSupplier;
        this.targetPose = targetPoseSupplier;
        m_swerve = swerve;
        addRequirements(swerve);

        rotationController = new PIDController(
                Drive.PID_DRIVE_ROTATION_KP.getValue(),
                Drive.PID_DRIVE_ROTATION_KI.getValue(),
                Drive.PID_DRIVE_ROTATION_KD.getValue());

        rotationController.enableContinuousInput(0, 2 * Math.PI);

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
        yDriveController.setTolerance(Drive.PID_DRIVE_Y_BARGE_TOLERANCE.getValue());
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
            yDriveController.setTolerance(Drive.PID_DRIVE_Y_BARGE_TOLERANCE.getValue());
            yDriveController.setPID(
                Drive.PID_DRIVE_Y_KP.getValue(),
                Drive.PID_DRIVE_Y_KI.getValue(),
                Drive.PID_DRIVE_Y_KD.getValue());
        }

        Pose2d currentPose = this.currentPose.get();
        Pose2d targetPose = this.targetPose.get();

        // Accounting for most efficient way to turn
        double drCC = rotationController.calculate(targetPose.getRotation().getRadians(), currentPose.getRotation().getRadians());
        double drCCW = rotationController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        
        double dr = Math.abs(drCC) > Math.abs(drCCW) ? drCCW: drCC;
        double dx = xDriveController.calculate(currentPose.getTranslation().getX(), targetPose.getTranslation().getX());
        double dy = yDriveController.calculate(currentPose.getTranslation().getY(), targetPose.getTranslation().getY());

        dr = - MathUtil.clamp(dr, -Drive.PID_DRIVE_MAX_ROTATION_SPEED.getValue(), Drive.PID_DRIVE_MAX_ROTATION_SPEED.getValue());
        dx = MathUtil.clamp(dx, -Drive.PID_DRIVE_MAX_DRIVE_X_SPEED.getValue(), Drive.PID_DRIVE_MAX_DRIVE_X_SPEED.getValue()); 
        dy = MathUtil.clamp(dy, -Drive.PID_DRIVE_MAX_DRIVE_Y_SPEED.getValue(), Drive.PID_DRIVE_MAX_DRIVE_Y_SPEED.getValue()); 

        // Invert drive inputs if on red alliance since field centric
        if(!m_swerve.isBlueAlliance()) {
            dx*=-1;
            dy*=-1;
        }


        m_swerve.drive(dr, dx, dy, true, 1.0);
    }

    public boolean isFinished() {
        return rotationController.atSetpoint() && xDriveController.atSetpoint() && yDriveController.atSetpoint();
    }
}
