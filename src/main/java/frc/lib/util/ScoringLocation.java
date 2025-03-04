package frc.lib.util;

import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.FieldConstants;

/** Scoring Locations for the 2025 game Reefscape */
public class ScoringLocation {

    /** Reef locations */
    public static enum CoralLocation {
        // @formatter:off
        A(coralPose(Rotation2d.kZero, true, 0), 18),
        B(coralPose(Rotation2d.kZero, false, 0), 18),
        C(coralPose(Rotation2d.fromDegrees(60), true, 0), 17),
        D(coralPose(Rotation2d.fromDegrees(60), false, 0), 17),
        E(coralPose(Rotation2d.fromDegrees(120), true, 0), 22),
        F(coralPose(Rotation2d.fromDegrees(120), false, 0), 22),
        G(coralPose(Rotation2d.k180deg, true, 0), 21),
        H(coralPose(Rotation2d.k180deg, false, 0), 21),
        I(coralPose(Rotation2d.fromDegrees(240), true, -2.5), 20),
        J(coralPose(Rotation2d.fromDegrees(240), false, -1.5), 20),
        K(coralPose(Rotation2d.fromDegrees(300), true, 0), 19),
        L(coralPose(Rotation2d.fromDegrees(300), false, 0), 19);
        // @formatter:on

        public final Pose2d pose;
        public final int tag;

        private static Pose2d coralPose(Rotation2d direction, boolean offset, double customOffset) {
            return new Pose2d(FieldConstants.Reef.center
                .plus(new Translation2d(
                    FieldConstants.Reef.inscribedRadius.in(Meters)
                        + Constants.Swerve.bumperFront.in(Meters),
                    direction.plus(Rotation2d.k180deg)))
                .plus(new Translation2d(Units.inchesToMeters((offset ? 12 : 0) + customOffset),
                    direction.plus(Rotation2d.kCCW_90deg))),
                direction);
        }

        CoralLocation(Pose2d pose, int tag) {
            this.pose = pose;
            this.tag = tag;
            String name = this.toString();
            Logger.recordOutput("Reef/" + name, this.pose);
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

        KP0("Algae 1", Constants.Elevator.P0, true), KP1("Coral 1", Constants.Elevator.P1,
            false), KP2("Algae 2", Constants.Elevator.P2, true), KP3("Coral 3",
                Constants.Elevator.P3, false), KP4("Coral 4", Constants.Elevator.P4,
                    false), KP5("Barge", Constants.Elevator.P5, true);

        public final String displayName;
        public final Distance height;
        public final boolean isAlgae;

        Height(String displayName, Distance height, boolean isAlgae) {
            this.displayName = displayName;
            this.height = height;
            this.isAlgae = isAlgae;
        }

        public int toInt() {
            switch (this) {
                case KP0:
                    return 2;
                case KP1:
                    return 1;
                case KP2:
                    return 4;
                case KP3:
                    return 3;
                case KP4:
                    return 5;
                default:
                    return 0;
            }
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
