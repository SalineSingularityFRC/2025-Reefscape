package frc.robot.commands.driving;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.Constants.Drive;
import lib.vision.RealSenseCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A command that takes in a field relative pose and drives the robot to it via
 * PID
 */
public class DriveToReefPole extends Command {
    private final SwerveSubsystem m_swerve;
    private final RealSenseCamera camera;
    private PIDController xDriveController;
    
    public DriveToReefPole(SwerveSubsystem swerve, RealSenseCamera camera) {
        this.m_swerve = swerve;
        addRequirements(swerve);
        this.camera = camera;

        xDriveController = new PIDController(
                Drive.PID_DRIVE_X_KP.getValue(),
                Drive.PID_DRIVE_X_KP.getValue() * 0.01,
                Drive.PID_DRIVE_X_KD.getValue()); 

        xDriveController.setSetpoint(-0.04);
        xDriveController.setTolerance(0.005);
    }

    public void execute() {
        if (Drive.PID_DRIVE_TUNING.getValue() > 0) {
            xDriveController.setTolerance(Drive.PID_DRIVE_X_TOLERANCE.getValue());
            xDriveController.setPID(
                    Drive.PID_DRIVE_X_KP.getValue(),
                    Drive.PID_DRIVE_X_KP.getValue() * 0.01,
                    Drive.PID_DRIVE_X_KD.getValue());
        }

        Pose2d targetPose = camera.getFinalReefPose().get();
        if (targetPose == null) {
            SmartDashboard.putString("DriveToReefPole/targetPose", "null");
            return;
        }
        SmartDashboard.putString("DriveToReefPole/targetPose", "valid");

        double dx = xDriveController.calculate(targetPose.getX(), -0.04);

        dx = MathUtil.clamp(dx, -Drive.PID_DRIVE_MAX_DRIVE_X_SPEED.getValue(),
                Drive.PID_DRIVE_MAX_DRIVE_X_SPEED.getValue());

        m_swerve.drive(0, 0, dx, false, 1.0);
    }

    public boolean isFinished() {
        return xDriveController.atSetpoint();
    }
}
