package frc.lib.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Scoring Locations for the 2025 game Reefscape */
public class ScoringLocation {

    /** Reef locations */
    public static enum CoralLocation {
        A, B, C, D, E, F, G, H, I, J, K, L
    }

    /**
     * set of height modes
     */
    public enum HeightMode {
        kAlgae("Algae"), kCoral("Coral");

        public final String displayName;

        HeightMode(String displayName) {
            this.displayName = displayName;
        }

        public static Trigger coralMode =
            new Trigger(() -> getCurrentHeightMode() == HeightMode.kCoral);
        public static Trigger algaeMode =
            new Trigger(() -> getCurrentHeightMode() == HeightMode.kAlgae);

        /**
         *
         * @return increments state
         */
        public HeightMode increment() {
            int new_ordinal = this.ordinal() + 1;
            if (new_ordinal >= HeightMode.values().length) {
                return this;
            }
            return HeightMode.values()[new_ordinal];
        }

        /**
         *
         * @return decrements state
         */
        public HeightMode decrement() {
            int new_ordinal = this.ordinal() - 1;
            if (new_ordinal < 0) {
                return this;
            }
            return HeightMode.values()[new_ordinal];
        }

        public static HeightMode currentState = HeightMode.kCoral;

        /** Advance state forward by one. */
        public static void incrementState() {
            currentState = currentState.increment();
        }

        /** Advance state backwards by one. */
        public static void decrementState() {
            currentState = currentState.decrement();
        }

        /** Get currently tracked state. */
        public static HeightMode getCurrentHeightMode() {
            return currentState;
        }
    }

    /**
     * algae height states
     */
    public enum AlgaeHeight {
        Klevel1("Lower"), Klevel2("Upper");

        public final String displayName;

        AlgaeHeight(String displayName) {
            this.displayName = displayName;
        }

        public static Trigger level1 =
            new Trigger(() -> getCurrentHeightMode() == AlgaeHeight.Klevel1);
        public static Trigger level2 =
            new Trigger(() -> getCurrentHeightMode() == AlgaeHeight.Klevel2);

        /**
         *
         * @return increments aglae state
         */
        public AlgaeHeight increment() {
            int new_ordinal = this.ordinal() + 1;
            if (new_ordinal >= AlgaeHeight.values().length) {
                return this;
            }
            return AlgaeHeight.values()[new_ordinal];
        }

        /**
         *
         * @return decrements algae state
         */
        public AlgaeHeight decrement() {
            int new_ordinal = this.ordinal() - 1;
            if (new_ordinal < 0) {
                return this;
            }
            return AlgaeHeight.values()[new_ordinal];
        }

        public static AlgaeHeight currentState = AlgaeHeight.Klevel1;

        /** Advance state forward by one. */
        public static void incrementState() {
            currentState = currentState.increment();
        }

        /** Advance state backwards by one. */
        public static void decrementState() {
            currentState = currentState.decrement();
        }

        public static AlgaeHeight getCurrentHeightMode() {
            return currentState;
        }
    }

    /**
     * Coral height states
     */
    public enum CoralHeight {
        Klevel1("level 1"), Klevel2("level 2"), Klevel3("level 3"), Klevel4("level 4");

        public final String displayName;

        CoralHeight(String displayName) {
            this.displayName = displayName;
        }

        public static Trigger level1 = new Trigger(() -> getCurrentState() == CoralHeight.Klevel1);
        public static Trigger level2 = new Trigger(() -> getCurrentState() == CoralHeight.Klevel2);
        public static Trigger level3 = new Trigger(() -> getCurrentState() == CoralHeight.Klevel3);
        public static Trigger level4 = new Trigger(() -> getCurrentState() == CoralHeight.Klevel4);

        /**
         *
         * @return increments coral state
         */
        public CoralHeight increment() {
            int new_ordinal = this.ordinal() + 1;
            if (new_ordinal >= CoralHeight.values().length) {
                return this;
            }
            return CoralHeight.values()[new_ordinal];
        }

        /**
         *
         * @return decrements coral state
         */
        public CoralHeight decrement() {
            int new_ordinal = this.ordinal() - 1;
            if (new_ordinal < 0) {
                return this;
            }
            return CoralHeight.values()[new_ordinal];

        }

        public static CoralHeight currentState = CoralHeight.Klevel1;

        /** Get currently tracked state. */
        public static CoralHeight getCurrentState() {
            return currentState;
        }

        /** Advance state forward by one. */
        public static void incrementState() {
            currentState = currentState.increment();
        }

        /** Advance state backwards by one. */
        public static void decrementState() {
            currentState = currentState.decrement();
        }
    }



}
