package lib.pose;

import frc.robot.subsystems.ElevatorSubsystem.Setpoint;

/**
 * Configuration class for scoring-related enums used in autonomous routines.
 * <p>
 * This final utility class defines the reef facets, target objects, and
 * comprehensive
 * target states that combine side, object type, and elevator setpoint for
 * scoring or intake actions.
 */
public final class ScoreConfig {
    /**
     * Private constructor to prevent instantiation of this utility class.
     */
    private ScoreConfig() {
        /* prevent instantiation */
    }

    /**
     * Enumeration of the reef facets on the field where the robot can score or
     * interact.
     */
    public enum FacetSide {
        /** The left side of the reef structure. */
        LEFT,
        /** The middle or center portion of the reef structure. */
        MIDDLE,
        /** The right side of the reef structure. */
        RIGHT;
    }

    /**
     * Defines the set of key autonomous navigation goals the robot can drive to.
     * <p>
     * Each constant represents either a specific game piece location (e.g., ALGAE,
     * CORAL),
     * an intake source (e.g., CORAL_SOURCE), or a positional waypoint relative to
     * field elements
     * (e.g., CLOSE_TO_BARGE, BARGE). These targets are used by drive routines to
     * select and
     * follow the appropriate PathPlanner trajectories or PID setpoints based on the
     * current
     * autonomous task and used in TargetState.
     * </p>
     */
    public enum NavigationTarget {
        /** Represents the algae game piece. */
        ALGAE,
        /** Represents backing away from the reef after intaking algae. */
        ALGAE_BACK_AWAY,
        /** Represents the coral game piece placed on the reef. */
        CORAL,
        /** Represents the coral source objects used for intake routines. */
        CORAL_SOURCE,
        /** Represents a placeholder for PIDing to a pose close to the barge. */
        CLOSE_TO_BARGE,
        /** Represents a placeholder for PIDing to the barge. */
        BARGE;
    }

    /**
     * Detailed target states combining facet side, object type, and elevator
     * setpoint.
     * <p>
     * Each enum constant specifies where on the field to position the robot and
     * which elevator setpoint to use for scoring or manipulation.
     */
    public enum TargetState {
        // ----- Coral scoring (button-press presets) -----
        /** Level 4 coral scoring on the left side. */
        L4_LEFT(FacetSide.LEFT, NavigationTarget.CORAL, Setpoint.kLevel4),
        /** Level 4 coral scoring on the right side. */
        L4_RIGHT(FacetSide.RIGHT, NavigationTarget.CORAL, Setpoint.kLevel4),
        /** Level 3 coral scoring on the left side. */
        L3_LEFT(FacetSide.LEFT, NavigationTarget.CORAL, Setpoint.kLevel3),
        /** Level 3 coral scoring on the right side. */
        L3_RIGHT(FacetSide.RIGHT, NavigationTarget.CORAL, Setpoint.kLevel3),
        /** Level 2 coral scoring on the left side. */
        L2_LEFT(FacetSide.LEFT, NavigationTarget.CORAL, Setpoint.kLevel2),
        /** Level 2 coral scoring on the right side. */
        L2_RIGHT(FacetSide.RIGHT, NavigationTarget.CORAL, Setpoint.kLevel2),
        /** Level 1 coral scoring (feeder station) on the left side. */
        L1_LEFT(FacetSide.LEFT, NavigationTarget.CORAL, Setpoint.kFeederStation),
        /** Level 1 coral scoring (feeder station) on the right side. */
        L1_RIGHT(FacetSide.RIGHT, NavigationTarget.CORAL, Setpoint.kFeederStation),

        // ----- General coral positioning (no fixed setpoint) -----
        /** General coral state on the left side (setpoint added elsewhere). */
        CORAL_LEFT(FacetSide.LEFT, NavigationTarget.CORAL, null),
        /** General coral state on the right side (setpoint added elsewhere). */
        CORAL_RIGHT(FacetSide.RIGHT, NavigationTarget.CORAL, null),

        // ----- Algae scoring/intake (button-press presets) -----
        /** General algae state at middle facet (setpoint added elsewhere). */
        ALGAE_BUTTON(FacetSide.MIDDLE, NavigationTarget.ALGAE, null),

        // ----- Algae positioning with elevator setpoints -----
        /** Lower algae position (feeder station height). */
        ALGAE_LOWER(FacetSide.MIDDLE, NavigationTarget.ALGAE, Setpoint.kFeederStation),
        /** Upper algae position (level 2 height). */
        ALGAE_UPPER(FacetSide.MIDDLE, NavigationTarget.ALGAE, Setpoint.kLevel2),
        /** For backing away after intaking algae. */
        ALGAE_BACK_AWAY(FacetSide.MIDDLE, NavigationTarget.ALGAE_BACK_AWAY, null),

        // ----- Coral source intake positions -----
        /** Coral source intake on the left side (feeder station). */
        LEFT_SOURCE(FacetSide.LEFT, NavigationTarget.CORAL_SOURCE, Setpoint.kFeederStation),
        /** Coral source intake on the right side (feeder station). */
        RIGHT_SOURCE(FacetSide.RIGHT, NavigationTarget.CORAL_SOURCE, Setpoint.kFeederStation);

        /** The reef facet side associated with this state. */
        private final FacetSide side;
        /** The type of game object for this state. */
        private final NavigationTarget object;
        /** The elevator setpoint, or null if no fixed setpoint required. */
        private final Setpoint setpoint;

        /**
         * Constructs a TargetState enum constant.
         *
         * @param side     the reef facet side where this state applies
         * @param object   the game object type for this state
         * @param setpoint the elevator subsystem setpoint, or null for dynamic control
         */
        TargetState(FacetSide side, NavigationTarget object, Setpoint setpoint) {
            this.side = side;
            this.object = object;
            this.setpoint = setpoint;
        }

        /**
         * Returns the reef facet side for this target state.
         *
         * @return the FacetSide enum value
         */
        public FacetSide getSide() {
            return side;
        }

        /**
         * Returns the game object type for this target state.
         *
         * @return the TargetObject enum value
         */
        public NavigationTarget getObject() {
            return object;
        }

        /**
         * Returns the elevator setpoint for this state.
         * May be null if dynamic or handled elsewhere.
         *
         * @return the elevator Setpoint, or null
         */
        public Setpoint getSetpoint() {
            return setpoint;
        }
    }
}
