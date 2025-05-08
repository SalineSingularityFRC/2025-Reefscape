package lib.pose;

import edu.wpi.first.math.geometry.Pose2d;
import lib.pose.ScoreConfig.FacetSide;
import lib.pose.ScoreConfig.NavigationTarget;
import lib.pose.ScoreConfig.TargetState;

/**
 * Represents a general robot pose in 2D space, along with the associated target
 * state
 * for autonomous operations.
 * <p>
 * A GeneralPose bundles together a human-readable name, a 2D pose (position and
 * orientation),
 * and a TargetState enum describing the desired mechanism setpoint or game
 * piece state.
 * This class is used by higher-level routines to drive the robot to specific
 * field locations
 * and perform actions (e.g., scoring coral or intaking algae) based on the
 * TargetState.
 */
public final class GeneralPose {
    /**
     * The name of this pose (for logging or dashboard display).
     */
    private final String m_name;

    /**
     * The 2D pose of the robot (x, y, rotation).
     */
    private Pose2d m_pose;

    /**
     * The desired target state associated with this pose (e.g., scoring height).
     */
    private TargetState m_targetState;

    /**
     * Constructs a new GeneralPose with the given name, pose, and target state.
     *
     * @param name        a non-null, human-readable identifier for this pose
     * @param pose        a non-null Pose2d representing the desired robot position
     *                    and orientation
     * @param targetState a non-null TargetState indicating the mechanism setpoint
     *                    or game piece state
     * @throws NullPointerException if name or pose are null
     */
    public GeneralPose(String name, Pose2d pose, TargetState targetState) {
        if (name == null) {
            throw new NullPointerException("Name parameter must not be null");
        }
        if (pose == null) {
            throw new NullPointerException("Pose2d parameter must not be null");
        }
        this.m_name = name;
        this.m_pose = pose;
        this.m_targetState = targetState;
    }

    /**
     * Gets the name of this pose.
     *
     * @return the human-readable name of the pose
     */
    public String getName() {
        return m_name;
    }

    /**
     * Gets the Pose2d (position and rotation) of this GeneralPose.
     *
     * @return the current Pose2d
     */
    public Pose2d getPose2d() {
        return m_pose;
    }

    /**
     * Creates a new GeneralPose with updated Pose2d
     *
     * @param m_pose a Pose2d to update the robot's target pose
     */
    public GeneralPose withPose2d(Pose2d newPose) {
        return new GeneralPose(m_name, newPose, m_targetState);
    }

    /**
     * Gets the TargetState associated with this pose.
     *
     * @return the current TargetState
     */
    public TargetState getTargetState() {
        return m_targetState;
    }

    /**
     * Gets the TargetObject associated with this pose.
     *
     * @return the current TargetObject
     */
    public NavigationTarget getObject() {
        return m_targetState.getObject();
    }

    /**
     * Gets the FacetSide associated with this pose.
     *
     * @return the current FacetSide
     */
    public FacetSide getSide() {
        return m_targetState.getSide();
    }

    /**
     * Sets a new TargetState for this GeneralPose.
     *
     * @param TargetState a TargetState enum defining the desired mechanism state
     */
    public GeneralPose withTargetState(TargetState newTargetState) {
        return new GeneralPose(m_name, m_pose, newTargetState);
    }
}
