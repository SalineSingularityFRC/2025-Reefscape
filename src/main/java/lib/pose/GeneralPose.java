package lib.pose;

import edu.wpi.first.math.geometry.Pose2d;
import lib.pose.ScoreConfig.TargetState;

/**
 * Represents a general robot pose in 2D space, along with the associated target state
 * for autonomous operations.
 * <p>
 * A GeneralPose bundles together a human-readable name, a 2D pose (position and orientation),
 * and a TargetState enum describing the desired mechanism setpoint or game piece state.
 * This class is used by higher-level routines to drive the robot to specific field locations
 * and perform actions (e.g., scoring coral or intaking algae) based on the TargetState.
 */
public class GeneralPose {
    /**
     * The name of this pose (for logging or dashboard display).
     */
    private final String name;

    /**
     * The 2D pose of the robot (x, y, rotation).
     */
    private Pose2d pose;

    /**
     * The desired target state associated with this pose (e.g., scoring height).
     */
    private TargetState targetState;

    /**
     * Constructs a new GeneralPose with the given name, pose, and target state.
     *
     * @param name         a non-null, human-readable identifier for this pose
     * @param pose         a non-null Pose2d representing the desired robot position and orientation
     * @param targetState  a non-null TargetState indicating the mechanism setpoint or game piece state
     * @throws NullPointerException if name or pose are null
     */
    public GeneralPose(String name, Pose2d pose, TargetState targetState) {
        if (name == null) {
            throw new NullPointerException("Name parameter must not be null");
        }
        if (pose == null) {
            throw new NullPointerException("Pose2d parameter must not be null");
        }
        this.name = name;
        this.pose = pose;
        this.targetState = targetState;
    }

    /**
     * Gets the name of this pose.
     *
     * @return the human-readable name of the pose
     */
    public String getName() {
        return name;
    }

    /**
     * Gets the Pose2d (position and rotation) of this GeneralPose.
     *
     * @return the current Pose2d
     */
    public Pose2d getPose2d() {
        return pose;
    }

    /**
     * Sets a new 2D pose for this GeneralPose.
     *
     * @param pose a non-null Pose2d to update the robot's target pose
     * @throws NullPointerException if pose is null
     */
    public void setPose2d(Pose2d pose) {
        if (pose == null) {
            throw new NullPointerException("Pose2d parameter must not be null");
        }
        this.pose = pose;
    }

    /**
     * Gets the TargetState associated with this pose.
     *
     * @return the current TargetState
     */
    public TargetState getTargetState() {
        return targetState;
    }

    /**
     * Sets a new TargetState for this GeneralPose.
     *
     * @param targetState a TargetState enum defining the desired mechanism state
     */
    public void setTargetState(TargetState targetState) {
        this.targetState = targetState;
    }
}
