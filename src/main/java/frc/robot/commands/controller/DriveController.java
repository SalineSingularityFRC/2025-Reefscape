package frc.robot.commands.controller;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveController extends Command {
    private final SwerveSubsystem m_swerve;
    private final DoubleSupplier m_rotation, m_x, m_y;
    private double multiplier;

    public DriveController(SwerveSubsystem swerve, DoubleSupplier rotation, DoubleSupplier x, DoubleSupplier y,
            double multiplier) {
        this.multiplier = multiplier;
        m_swerve = swerve;
        m_rotation = rotation;
        m_x = x;
        m_y = y;
        addRequirements(swerve);
    }
    
    public void execute() {
        // SmartDashboard.putNumber("Input X", fixDecimalTo2Places(-m_x.getAsDouble()));
        // SmartDashboard.putNumber("Input Y", fixDecimalTo2Places(-m_y.getAsDouble()));
        // SmartDashboard.putNumber("Input Rotation", fixDecimalTo2Places(-m_rotation.getAsDouble()));
        m_swerve.drive(
                -m_rotation.getAsDouble(),
                -m_x.getAsDouble(),
                -m_y.getAsDouble(),
                true,
                multiplier);
    }

    public boolean isFinished() {
        return false;
    }
}
