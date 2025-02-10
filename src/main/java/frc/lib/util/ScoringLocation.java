package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

/** Scoring Locations for the 2025 game Reefscape */
public class ScoringLocation {

    /** Reef locations */
    public static enum CoralLocation {
        A(new Pose2d()), B(new Pose2d()), C(new Pose2d()), D(new Pose2d()), E(new Pose2d()), F(
            new Pose2d()), G(new Pose2d()), H(
                new Pose2d()), I(new Pose2d()), J(new Pose2d()), K(new Pose2d()), L(new Pose2d());

        public final Pose2d targetPose;

        CoralLocation(Pose2d targetPose) {
            this.targetPose = targetPose;
        }

        public static CoralLocation fromChar(char c) {
            switch (c) {
                case 'a':
                    return CoralLocation.A;
                case 'b':
                    return CoralLocation.B;
                case 'c':
                    return CoralLocation.C;
                case 'd':
                    return CoralLocation.D;
                case 'e':
                    return CoralLocation.E;
                case 'f':
                    return CoralLocation.F;
                case 'g':
                    return CoralLocation.G;
                case 'h':
                    return CoralLocation.H;
                case 'i':
                    return CoralLocation.I;
                case 'j':
                    return CoralLocation.J;
                case 'k':
                    return CoralLocation.K;
                case 'l':
                    return CoralLocation.L;
                default:
                    return null;
            }
        }

        @Override
        public String toString() {
            switch (this) {
                case A:
                    return "a";
                case B:
                    return "b";
                case C:
                    return "c";
                case D:
                    return "d";
                case E:
                    return "e";
                case F:
                    return "f";
                case G:
                    return "g";
                case H:
                    return "h";
                case I:
                    return "i";
                case J:
                    return "j";
                case K:
                    return "k";
                case L:
                    return "l";
                default:
                    return "?";

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
        Klevel2, Klevel3;

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

        public static AlgaeHeight currentState = AlgaeHeight.Klevel2;

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

        public static AlgaeHeight fromChar(char c) {
            switch (c) {
                case '2':
                    return Klevel2;
                case '3':
                    return Klevel3;
                default:
                    return null;
            }
        }

        @Override
        public String toString() {
            switch (this) {
                case Klevel2:
                    return "2";
                case Klevel3:
                    return "3";
                default:
                    return "?";

            }
        }
    }

    /**
     * Coral height states
     */
    public enum CoralHeight {
        Klevel1(Height.kHome), Klevel2(Height.KPosition1), Klevel3(Height.KPosition3), Klevel4(
            Height.kPosition4);

        public final Height height;

        CoralHeight(Height height) {
            this.height = height;
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

        public static CoralHeight fromChar(char c) {
            switch (c) {
                case '1':
                    return Klevel1;
                case '2':
                    return Klevel2;
                case '3':
                    return Klevel3;
                case '4':
                    return Klevel4;
                default:
                    return null;
            }
        }

        @Override
        public String toString() {
            switch (this) {
                case Klevel1:
                    return "1";
                case Klevel2:
                    return "2";
                case Klevel3:
                    return "3";
                case Klevel4:
                    return "4";
                default:
                    return "?";

            }
        }
    }

    /**
     * Coral height states
     */
    public enum Height {
        kHome("Home", Constants.Elevator.HOME), KPosition0("P0", Constants.Elevator.P0), KPosition1(
            "P1", Constants.Elevator.P1), KPosition2("P2", Constants.Elevator.P2), KPosition3("P3",
                Constants.Elevator.P3), kPosition4("P4", Constants.Elevator.P4);

        public final String name;
        public final Distance height;

        Height(String name, Distance height) {
            this.name = name;
            this.height = height;
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

    public enum AlgaeLocation {
        Processor, Barge, Drop;

        public static AlgaeLocation fromChar(char c) {
            switch (c) {
                case 'p':
                    return Processor;
                case 'b':
                    return Barge;
                case 'd':
                    return Drop;
                default:
                    return null;
            }
        }

        @Override
        public String toString() {
            switch (this) {
                case Processor:
                    return "p";
                case Barge:
                    return "b";
                case Drop:
                    return "d";
                default:
                    return "?";

            }
        }
    }

}
