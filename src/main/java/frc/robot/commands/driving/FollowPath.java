package frc.robot.commands.driving;

import java.io.IOException;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Drive.PathFinding;
import frc.robot.subsystems.SwerveSubsystem;
import lib.pose.GeneralPose;
import lib.pose.ScoreConfig.NavigationTarget;

/**
 * A Command that loads a PathPlanner path based on the robot’s current target
 * and then delegates execution to the generated path-follower command.
 */
public class FollowPath extends Command {
    // Supplies the current robot pose and associated target object
    private final Supplier<GeneralPose> m_generalPoseSupplier;
    // The PathPlanner path we’ll load dynamically
    private PathPlannerPath m_chosenPath;
    // The internal command that actually follows the trajectory
    private Command m_delegate;
    // Flag to indicate if loading the path failed
    private boolean m_failedToLoadPath;
    // Create the constraints to use while pathfinding
    private PathConstraints m_constraints;
    // The swerve susbsytem for updating rotational PID after interupt/end
    private SwerveSubsystem m_swerveSubsystem;

    /**
     * @param generalPoseSupplier A supplier that returns the target GeneralPose
     * @param swerve              The swerve subsystem for resetting rotational
     *                            control if command in interrupted
     */
    public FollowPath(Supplier<GeneralPose> generalPoseSupplier, SwerveSubsystem swerve) {
        m_generalPoseSupplier = generalPoseSupplier;
        m_swerveSubsystem = swerve;
        m_constraints = new PathConstraints(
                PathFinding.maxVelocityMPS.getValue(),
                PathFinding.maxAccelerationMPSSq.getValue(),
                Units.degreesToRadians(PathFinding.maxAngularVelocityDegPerSec.getValue()),
                Units.degreesToRadians(PathFinding.maxAngularAccelerationDegPerSecSq.getValue()));

        addRequirements(m_swerveSubsystem);
    }

    private boolean loadPathWithPrefix(String pathPrefix, String poseName) {
        try {
            String pathName = pathPrefix + " " + poseName;
            m_chosenPath = PathPlannerPath.fromPathFile(pathName);
            return true;
        } catch (IOException | org.json.simple.parser.ParseException | FileVersionException e) {
            String pathName = pathPrefix + " " + poseName;
            DriverStation.reportError(
                    "Failed to load path \"" + pathName + "\": " + e.getMessage(),
                    e.getStackTrace());
            return false;
        }
    }

    @Override
    public void initialize() {
        // 1) Fetch the name & target object from the target GeneralPose
        String generalPoseName = m_generalPoseSupplier.get().getName();
        NavigationTarget navTarget = m_generalPoseSupplier.get().getNavTarget();

        // Capture the most recent changes from Glass
        m_constraints = new PathConstraints(
                PathFinding.maxVelocityMPS.getValue(),
                PathFinding.maxAccelerationMPSSq.getValue(),
                Units.degreesToRadians(PathFinding.maxAngularVelocityDegPerSec.getValue()),
                Units.degreesToRadians(PathFinding.maxAngularAccelerationDegPerSecSq.getValue()));

        String pathPrefix = switch (navTarget) {
            case ALGAE -> "To Algae";
            case ALGAE_BACK_AWAY -> "Back Away From Algae";
            default -> null;
        };

        if (pathPrefix != null) {
            m_failedToLoadPath = !loadPathWithPrefix(pathPrefix, generalPoseName);
        } else {
            m_delegate = Commands.none();
        }

        // 3) If loading succeeded, create the path-follower command; otherwise do
        // nothing
        if (!m_failedToLoadPath) {
            if (navTarget == NavigationTarget.ALGAE_BACK_AWAY) {
                m_delegate = AutoBuilder.followPath(m_chosenPath);
            } else {
                m_delegate = AutoBuilder.pathfindThenFollowPath(m_chosenPath, m_constraints);
            }
        } else {
            m_delegate = Commands.none();
        }

        // 4) Initialize the delegated path-follower command
        m_delegate.initialize();

        // Reset for next time this command is run
        m_failedToLoadPath = false;
    }

    @Override
    public void execute() {
        // Delegate all periodic work to the path-follower
        m_delegate.execute();
    }

    @Override
    public boolean isFinished() {
        // Finish when the delegated command is done
        return m_delegate.isFinished();
    }

    // Don't schedule commands in end()
    @Override
    public void end(boolean interrupted) {
        // Ensure proper cleanup on interruption or normal end
        m_delegate.end(interrupted);
    }
}
