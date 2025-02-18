package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Scoring Locations for the 2025 game Reefscape */
public class ScoringLocation {

    /** Reef locations */
    public static enum CoralLocation {
        // @formatter:off
        A(new Pose2d(2.855360269546509, 4.392190456390381, Rotation2d.kZero), 18),
        B(new Pose2d(2.855360269546509, 4.014956951141357, Rotation2d.kZero), 18),
        C(new Pose2d(3.321354627609253, 2.822233200073242, Rotation2d.fromDegrees(60)), 17),
        D(new Pose2d(3.6763980388641357, 2.616973876953125, Rotation2d.fromDegrees(60)), 17),
        E(new Pose2d(4.935692310333252, 2.4061667919158936, Rotation2d.fromDegrees(120)), 22),
        F(new Pose2d(5.307377815246582, 2.616973876953125, Rotation2d.fromDegrees(120)), 22),
        G(new Pose2d(6.128415584564209, 3.64327073097229, Rotation2d.k180deg), 21),
        H(new Pose2d(6.122868061065674, 4.0537896156311035, Rotation2d.k180deg), 21),
        I(new Pose2d(5.623588562011719, 5.257608413696289, Rotation2d.fromDegrees(240)), 20),
        J(new Pose2d(5.296282768249512, 5.440677642822266, Rotation2d.fromDegrees(240)), 20),
        K(new Pose2d(4.025893688201904, 5.657032012939453, Rotation2d.fromDegrees(300)), 19),
        L(new Pose2d(3.6653027534484863, 5.440677642822266, Rotation2d.fromDegrees(300)), 19);
        // @formatter:on

        public final Pose2d pose;
        public final int tag;

        CoralLocation(Pose2d pose, int tag) {
            this.pose = pose;
            this.tag = tag;
        }

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
                default:
                    return null;
            }
        }
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

    /** All elevator heights (except home). */
    public enum ElevatorHeight {
        CoralL1, CoralL2, CoralL3, CoralL4, AlgaeL2, AlgaeL3;

        /** Deserialize from integer */
        public static ElevatorHeight fromInt(int i) {
            switch (i) {
                case 1:
                    return CoralL2;
                case 2:
                    return AlgaeL2;
                case 3:
                    return CoralL3;
                case 4:
                    return AlgaeL3;
                case 5:
                    return CoralL4;
                default:
                    return CoralL1;
            }
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



}
