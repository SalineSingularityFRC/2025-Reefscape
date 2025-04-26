package lib.pose;

import edu.wpi.first.math.geometry.Pose2d;
import lib.pose.ScoreConfig.TargetState;

public class GeneralPose {
    private final String name;
    private Pose2d pose;
    private TargetState targetState;

    /**
     * Creates a new GeneralPose.
     *
     * @param name the name; must not be null
     * @param side the facet side; must not be null
     * @param pose the 2D pose; must not be null
     */
    public GeneralPose(String name, Pose2d pose, TargetState targetState) {
        if (name == null || pose == null) {
            throw new NullPointerException("Name and Pose2d parameters must not be null");
        }
        this.name = name;
        this.pose = pose;
        this.targetState = targetState;
    }

    public String getName() {
        return name;
    }

    public Pose2d getPose2d() {
        return pose;
    }

    public TargetState getTargetState() {
        return targetState;
    }

    public void setTargetState(TargetState targetState) {
        this.targetState = targetState;
    }

    public void setPose2d(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public String toString() {
        return "GeneralPose[" +
                "name=" + name +
                ", pose=" + pose +
                ", setpoint=" + targetState +
                ']';
    }
}
