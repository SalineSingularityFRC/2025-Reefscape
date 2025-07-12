package frc.robot.commands.driving;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import lib.pose.ScoreConfig.NavigationTarget;

import static frc.robot.Constants.Drive;

/**
 * A command that takes in a field relative pose and drives the robot to it via
 * PID
 */
public class DriveToPose2d extends Command {
    private final SwerveSubsystem m_swerveSubsystem;
    private final Supplier<Pose2d> targetPose, currentPose;
    private PIDController rotationController;
    private PIDController xDriveController;
    private PIDController yDriveController;
    private NavigationTarget m_navigationTarget;

    public DriveToPose2d(SwerveSubsystem swerve, Supplier<Pose2d> currentPoseSupplier,
            Supplier<Pose2d> targetPoseSupplier, NavigationTarget navigationTarget) {
        this.currentPose = currentPoseSupplier;
        this.targetPose = targetPoseSupplier;
        m_swerveSubsystem = swerve;
        m_navigationTarget = navigationTarget;
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

        // Switch tolerance if barge scoring due to limelights updating pose near barge
        if (m_navigationTarget == NavigationTarget.BARGE) {
            yDriveController.setTolerance(Drive.PID_DRIVE_Y_BARGE_TOLERANCE.getValue());
        } else {
            yDriveController.setTolerance(Drive.PID_DRIVE_Y_TOLERANCE.getValue());
        }
    }

    @Override
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

            // Switch tolerance if barge scoring due to limelights updating pose near barge
            if (m_navigationTarget == NavigationTarget.BARGE) {
                yDriveController.setTolerance(Drive.PID_DRIVE_Y_BARGE_TOLERANCE.getValue());
            } else {
                yDriveController.setTolerance(Drive.PID_DRIVE_Y_TOLERANCE.getValue());
            }

            yDriveController.setPID(
                    Drive.PID_DRIVE_Y_KP.getValue(),
                    Drive.PID_DRIVE_Y_KI.getValue(),
                    Drive.PID_DRIVE_Y_KD.getValue());
        }

        Pose2d currentPose = this.currentPose.get();
        Pose2d targetPose = this.targetPose.get();

        double errorR = targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians();
        // errorR = ((errorR + Math.PI) % (2*Math.PI)) - Math.PI;
        // if (errorR > Math.PI) {
        // errorR =- 2 * Math.PI;
        // } else if( errorR < - Math.PI) {
        // errorR =+ 2* Math.PI;
        // }

        SmartDashboard.putNumber("Tuning/Error", errorR);

        // Accounting for most efficient way to turn
        double drCC = rotationController.calculate(targetPose.getRotation().getRadians(),
                currentPose.getRotation().getRadians());
        double drCCW = rotationController.calculate(currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians());
        SmartDashboard.putNumber("Tuning/Current Rot", currentPose.getRotation().getRadians());
        SmartDashboard.putNumber("Tuning/Targ rot", targetPose.getRotation().getRadians());

        double dr = Math.abs(drCC) > Math.abs(drCCW) ? drCCW : drCC;
        double dx = xDriveController.calculate(currentPose.getTranslation().getX(), targetPose.getTranslation().getX());
        double dy = yDriveController.calculate(currentPose.getTranslation().getY(), targetPose.getTranslation().getY());

        dr = -MathUtil.clamp(dr, -Drive.PID_DRIVE_MAX_ROTATION_SPEED.getValue(),
                Drive.PID_DRIVE_MAX_ROTATION_SPEED.getValue());

        // Clamp control effort based on target object (not used)
        double maxDriveXSpeed = m_navigationTarget == NavigationTarget.ALGAE ? Drive.PID_DRIVE_MAX_DRIVE_ALGAE_X_SPEED.getValue() : Drive.PID_DRIVE_MAX_DRIVE_X_SPEED.getValue();
        double maxDriveYSpeed = m_navigationTarget == NavigationTarget.ALGAE ? Drive.PID_DRIVE_MAX_DRIVE_ALGAE_Y_SPEED.getValue() : Drive.PID_DRIVE_MAX_DRIVE_Y_SPEED.getValue();
        dx = MathUtil.clamp(dx, -maxDriveXSpeed, maxDriveXSpeed);
        dy = MathUtil.clamp(dy, -maxDriveYSpeed, maxDriveYSpeed);

        // Invert drive inputs if on red alliance since field centric
        if (!m_swerveSubsystem.isBlueAlliance()) {
            dx *= -1;
            dy *= -1;
        }

        SmartDashboard.putNumber("Tuning/dr", dr);
        SmartDashboard.putNumber("Tuning/dx", dx);
        SmartDashboard.putNumber("Tuning/dy", dy);

        m_swerveSubsystem.drive(dr, dx, dy, true, 1.0);
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("Tuning/rotationController at setpoint", rotationController.atSetpoint());
        SmartDashboard.putBoolean("Tuning/xDriveController at setpoint", xDriveController.atSetpoint());
        SmartDashboard.putBoolean("Tuning/yDriveController at setpoint", yDriveController.atSetpoint());
        SmartDashboard.putBoolean("Tuning/Is command finished",
                rotationController.atSetpoint() && xDriveController.atSetpoint() && yDriveController.atSetpoint());
        return rotationController.atSetpoint() && xDriveController.atSetpoint() && yDriveController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.stopDriving();
        m_swerveSubsystem.updateRotationPIDSetpointCommand();
    }

}
