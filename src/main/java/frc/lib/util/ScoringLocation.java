package frc.lib.util;

/** Scoring Locations for the 2025 game Reefscape */
public class ScoringLocation {

    /** Reef locations */
    public static enum CoralLocation {
        A, B, C, D, E, F, G, H, I, J, K, L
    }

    public enum HeightMode {
        kAlgae, kCoral;

        public HeightMode increment() {
            int new_ordinal = this.ordinal() + 1;
            if (new_ordinal >= CoralHeight.values().length) {
                return this;
            }
            return HeightMode.values()[new_ordinal];
        }

        public HeightMode decrement() {
            int new_ordinal = this.ordinal() - 1;
            if (new_ordinal >= CoralHeight.values().length) {
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

    public enum AlgaeHeight {
        Klevel1, Klevel2;

        public AlgaeHeight increment() {
            int new_ordinal = this.ordinal() + 1;
            if (new_ordinal >= CoralHeight.values().length) {
                return this;
            }
            return AlgaeHeight.values()[new_ordinal];
        }

        public AlgaeHeight decrement() {
            int new_ordinal = this.ordinal() - 1;
            if (new_ordinal >= CoralHeight.values().length) {
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
    }
    public enum CoralHeight {
        Klevel1, Klevel2, Klevel3, Klevel4;

        public CoralHeight increment() {
            int new_ordinal = this.ordinal() + 1;
            if (new_ordinal >= CoralHeight.values().length) {
                return this;
            }
            return CoralHeight.values()[new_ordinal];
        }

        /** Get previous state. */
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
