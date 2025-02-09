package frc.lib.util;

/** Scoring Locations for the 2025 game Reefscape */
public class ScoringLocation {

    /** Reef locations */
    public static enum CoralLocation {
        A, B, C, D, E, F, G, H, I, J, K, L, LeftFeeder, RightFeeder, Processor;

        /** Deserialize from integer */
        public static CoralLocation fromInt(int id) {
            switch (id) {
                case 0:
                    return CoralLocation.A;
                case 1:
                    return CoralLocation.B;
                case 2:
                    return CoralLocation.C;
                case 3:
                    return CoralLocation.D;
                case 4:
                    return CoralLocation.E;
                case 5:
                    return CoralLocation.F;
                case 6:
                    return CoralLocation.G;
                case 7:
                    return CoralLocation.H;
                case 8:
                    return CoralLocation.I;
                case 9:
                    return CoralLocation.J;
                case 10:
                    return CoralLocation.K;
                case 11:
                    return CoralLocation.L;
                case 12:
                    return CoralLocation.LeftFeeder;
                case 13:
                    return CoralLocation.RightFeeder;
                case 14:
                    return CoralLocation.Processor;
                default:
                    return null;
            }
        }
    }

    /**
     * set of height modes
     */
    public enum HeightMode {
        kAlgae, kCoral;

        /**
         *
         * @return increments state
         */
        public HeightMode increment() {
            int new_ordinal = this.ordinal() + 1;
            if (new_ordinal >= Height.values().length) {
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
            if (new_ordinal >= Height.values().length) {
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
        Klevel1, Klevel2;

        /**
         *
         * @return increments aglae state
         */
        public AlgaeHeight increment() {
            int new_ordinal = this.ordinal() + 1;
            if (new_ordinal >= CoralHeight.values().length) {
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

        public static AlgaeHeight getCurrentHeightMode() {
            return currentState;
        }
    }

    /**
     * Coral height states
     */
    public enum CoralHeight {
        Klevel1, Klevel2, Klevel3, Klevel4;

        /** Deserialize from integer */
        public static CoralHeight fromInt(int id) {
            switch (id) {
                case 0:
                    return CoralHeight.Klevel1;
                case 1:
                    return CoralHeight.Klevel2;
                case 2:
                    return CoralHeight.Klevel3;
                case 3:
                    return CoralHeight.Klevel4;
                case 4:
                    return CoralHeight.Klevel4;
                default:
                    return null;
            }
        }

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

    /**
     * Coral height states
     */
    public enum Height {
        kHome("Home"), KPosition0("P0"), KPosition1("P1"), KPosition2("P2"), KPosition3(
            "P3"), kPosition4("P4");

        public final String name;

        Height(String name) {
            this.name = name;
        }

        /**
         *
         * @return increments coral state
         */
        public Height increment() {
            int new_ordinal = this.ordinal() + 1;
            if (new_ordinal >= Height.values().length) {
                return this;
            }
            return Height.values()[new_ordinal];
        }

        /**
         *
         * @return decrements coral state
         */
        public Height decrement() {
            int new_ordinal = this.ordinal() - 1;
            if (new_ordinal < 0) {
                return this;
            }
            return Height.values()[new_ordinal];

        }

        public static Height currentState = Height.kHome;

        /** Get currently tracked state. */
        public static Height getCurrentState() {
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
