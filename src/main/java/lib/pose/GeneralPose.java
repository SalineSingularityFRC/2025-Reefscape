package lib.pose;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.ElevatorSubsystem.Setpoint;
import lib.pose.ScoreConfig.FacetSide;

public class GeneralPose {
    private final String name;
    private final FacetSide side;
    private final Pose2d pose;
    private final Setpoint setpoint;

    /**
     * Creates a new GeneralPose.
     *
     * @param name the name; must not be null
     * @param side the facet side; must not be null
     * @param pose the 2D pose; must not be null
     */
    public GeneralPose(String name, FacetSide side, Pose2d pose, Setpoint setpoint) {
        if (name == null || side == null || pose == null) {
            throw new NullPointerException("GeneralPose parameters must not be null");
        }
        this.name = name;
        this.side = side;
        this.pose = pose;
        this.setpoint = setpoint;
    }

    public String getName() {
        return name;
    }

    public FacetSide getSide() {
        return side;
    }

    public Pose2d getPose() {
        return pose;
    }

    public Setpoint getSetpoint() {
        return setpoint;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o)
            return true;
        if (!(o instanceof GeneralPose))
            return false;
        GeneralPose that = (GeneralPose) o;
        return name.equals(that.name)
                && side == that.side
                && pose.equals(that.pose)
                && setpoint.equals(that.setpoint);
    }

    @Override
    public String toString() {
        return "GeneralPose[" +
                "name=" + name +
                ", side=" + side +
                ", pose=" + pose +
                ']';
    }
}
