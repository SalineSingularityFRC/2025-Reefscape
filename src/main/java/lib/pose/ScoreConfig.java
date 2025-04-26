package lib.pose;

/**
 * Container for scoring-related enums.
 */
public final class ScoreConfig {
    private ScoreConfig() {
        /* prevent instantiation */ }

    /** Indicates which facet of the reef to score on. */
    public enum FacetSide {
        LEFT, 
        RIGHT, 
        MIDDLE;
    }

    /** What kind of object the robot is scoring / going to. */
    public enum TargetObject {
        ALGAE, 
        CORAL, 
        CORAL_SOURCE;
    }

    public enum Target {
        // For Coral Scoring
        L4_LEFT(FacetSide.LEFT, TargetObject.CORAL),
        L4_RIGHT(FacetSide.RIGHT, TargetObject.CORAL),
        L3_LEFT(FacetSide.LEFT, TargetObject.CORAL),
        L3_RIGHT(FacetSide.RIGHT, TargetObject.CORAL),
        L2_LEFT(FacetSide.LEFT, TargetObject.CORAL),
        L2_RIGHT(FacetSide.RIGHT, TargetObject.CORAL),
        L1_LEFT(FacetSide.LEFT, TargetObject.CORAL),
        L1_RIGHT(FacetSide.RIGHT, TargetObject.CORAL),

        // For Algae
        L1_MIDDLE(FacetSide.MIDDLE, TargetObject.ALGAE),
        L2_MIDDLE(FacetSide.MIDDLE, TargetObject.ALGAE),

        // For Coral Source
        LEFT_SOURCE(FacetSide.LEFT, TargetObject.CORAL_SOURCE),
        RIGHT_SOURCE(FacetSide.RIGHT, TargetObject.CORAL_SOURCE);

        private final FacetSide side;
        public final TargetObject object;

        Target(FacetSide side, TargetObject object) {
            this.side = side;
            this.object = object;
        }

        public FacetSide getSide() {
            return side;
        }

        public TargetObject getObject() {
            return object;
        }
    }
}
