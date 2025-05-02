package frc.robot.commands.driving;

import java.io.IOException;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.pose.GeneralPose;
import lib.pose.ScoreConfig.TargetObject;

/**
 * A Command that loads a PathPlanner path based on the robot’s current target
 * and then delegates execution to the generated path‐follower command.
 */
public class FollowPath extends Command {
    // Supplies the current robot pose and associated target object
    private final Supplier<GeneralPose> m_generalPoseSupplier;
    // The PathPlanner path we’ll load dynamically
    private PathPlannerPath chosenPath;
    // The internal command that actually follows the trajectory
    private Command m_delegate;
    // Flag to indicate if loading the path failed
    private boolean failedToLoadPath;

    /**
     * @param generalPoseSupplier A supplier that returns the target GeneralPose
     */
    public FollowPath(Supplier<GeneralPose> generalPoseSupplier) {
        m_generalPoseSupplier = generalPoseSupplier;
    }

    @Override
    public void initialize() {
        // 1) Fetch the name & target object from the target GeneralPose
        String generalPoseName = m_generalPoseSupplier.get().getName();
        TargetObject targetObject = m_generalPoseSupplier.get().getObject();

        // 2) Only handle the ALGAE object for now
        if (targetObject == TargetObject.ALGAE) {
            try {
                // Build the path file name like "To Algae [PoseName].path"
                chosenPath = PathPlannerPath.fromPathFile("To Algae " + generalPoseName);
            } catch (IOException | org.json.simple.parser.ParseException | FileVersionException e) {
                // Report any file‐loading or parsing errors to DriverStation
                DriverStation.reportError(
                    "Failed to load path \"" + "To Algae " + generalPoseName + "\": " + e.getMessage(),
                    e.getStackTrace()
                );
                failedToLoadPath = true;
            }
        } else {
            // If not ALGAE, do nothing — fallback to an empty command
            m_delegate = Commands.none();
        }
        
        // 3) If loading succeeded, create the follower; otherwise use a no‐op
        if (!failedToLoadPath) {
            m_delegate = AutoBuilder.followPath(chosenPath);
        } else {
            m_delegate = Commands.none();
        }

        // 4) Initialize the delegated path‐follower command
        m_delegate.initialize();

        // Reset for next time this command is run
        failedToLoadPath = false;
    }

    @Override
    public void execute() {
        // Delegate all periodic work to the path‐follower
        m_delegate.execute();
    }

    @Override
    public boolean isFinished() {
        // Finish when the delegated command is done
        return m_delegate.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        // Ensure proper cleanup on interruption or normal end
        m_delegate.end(interrupted);
    }
}
