package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class CameraDriveToPose extends Command {
    private final SwerveSubsystem m_swerve;
    private final Supplier<Pose3d> cameraReefPose, cameraPose;
    private double multiplier;

    public CameraDriveToPose(SwerveSubsystem swerve, Supplier<Pose3d> cameraReefPose, Supplier<Pose3d> cameraPose,
      Matrix<N3, N1> normalVector, double targetAngle) {
        this.cameraPose = cameraPose;
        this.cameraReefPose = cameraReefPose;
        m_swerve = swerve;
        addRequirements(swerve);
    }

    public void execute() {

    }

    public boolean isFinished() {
        return false;
    }
}
