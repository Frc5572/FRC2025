package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

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
    public enum Height {
        KP0("Algae 1", Constants.Elevator.P0), KP1("Coral 2", Constants.Elevator.P1), KP2("Algae 2",
            Constants.Elevator.P2), KP3("Coral 3",
                Constants.Elevator.P3), KP4("Coral 4", Constants.Elevator.P4);

        public final String displayName;
        public final Distance height;

        Height(String displayName, Distance height) {
            this.displayName = displayName;
            this.height = height;
        }

        /** Deserialize from integer */
        public static Height fromInt(int value) {
            switch (value) {
                case 0:
                    return KP0;
                case 1:
                    return KP1;
                case 2:
                    return KP0;
                case 3:
                    return KP3;
                case 4:
                    return KP2;
                case 5:
                    return KP4;
                default:
                    return Height.KP0;
            }
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

        public static Height currentState = Height.KP1;

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
