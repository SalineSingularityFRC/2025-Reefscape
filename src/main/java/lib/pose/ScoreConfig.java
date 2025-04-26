package lib.pose;

import frc.robot.subsystems.ElevatorSubsystem.Setpoint;

/**
 * Container for scoring-related enums.
 */
public final class ScoreConfig {
    private ScoreConfig() {
        /* prevent instantiation */ }

    /** Indicates which facet of the reef to score on. */
    public enum FacetSide {
        LEFT,
        MIDDLE,
        RIGHT;
    }

    /** The object the robot is scoring / going to. */
    public enum TargetObject {
        ALGAE, 
        CORAL, 
        CORAL_SOURCE;
    }

    /** Goal state */
    public enum TargetState {

        // For coral scoring (buttons)
        L4_LEFT(FacetSide.LEFT, TargetObject.CORAL, Setpoint.kLevel4),
        L4_RIGHT(FacetSide.RIGHT, TargetObject.CORAL, Setpoint.kLevel4),
        L3_LEFT(FacetSide.LEFT, TargetObject.CORAL, Setpoint.kLevel3),
        L3_RIGHT(FacetSide.RIGHT, TargetObject.CORAL, Setpoint.kLevel3),
        L2_LEFT(FacetSide.LEFT, TargetObject.CORAL, Setpoint.kLevel2),
        L2_RIGHT(FacetSide.RIGHT, TargetObject.CORAL, Setpoint.kLevel2),
        L1_LEFT(FacetSide.LEFT, TargetObject.CORAL, Setpoint.kFeederStation),
        L1_RIGHT(FacetSide.RIGHT, TargetObject.CORAL, Setpoint.kFeederStation),

        // For general side coral (general pose)
        CORAL_LEFT(FacetSide.LEFT, TargetObject.CORAL, null),
        CORAL_RIGHT(FacetSide.RIGHT, TargetObject.CORAL, null),

        // For algae (buttons)
        ALGAE_BUTTON(FacetSide.MIDDLE, TargetObject.ALGAE, null),

        // For algae (general pose)
        ALGAE_LOWER(FacetSide.MIDDLE, TargetObject.ALGAE, Setpoint.kFeederStation),
        ALGAE_UPPER(FacetSide.MIDDLE, TargetObject.ALGAE, Setpoint.kLevel2),

        // For Coral Source
        LEFT_SOURCE(FacetSide.LEFT, TargetObject.CORAL_SOURCE, Setpoint.kFeederStation),
        RIGHT_SOURCE(FacetSide.RIGHT, TargetObject.CORAL_SOURCE, Setpoint.kFeederStation);

        private final FacetSide side;
        private final TargetObject object;
        private final Setpoint setpoint;

        TargetState(FacetSide side, TargetObject object, Setpoint setpoint) {
            this.side = side;
            this.object = object;
            this.setpoint = setpoint;
        }

        public FacetSide getSide() {
            return side;
        }

        public TargetObject getObject() {
            return object;
        }

        public Setpoint getSetpoint() {
            return setpoint;
        }
    }
}
