package frc.lib.util;

import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

/** Scoring Locations for the 2025 game Reefscape */
public class ScoringLocation {

    /** Reef locations */
    public static enum CoralLocation {
        A, B, C, D, E, F, G, H, I, J, K, L
    }

    /**
     * set of height modes
     */
    public enum Height {
        KP0("Algae 1", Constants.Elevator.P0), KP1("Coral 1", Constants.Elevator.P1), KP2("Algae 2",
            Constants.Elevator.P2), KP3("Coral 3", Constants.Elevator.P3), KP4("Coral 4",
                Constants.Elevator.P4), KP5("Barge", Constants.Elevator.P5);

        public final String displayName;
        public final Distance height;

        Height(String displayName, Distance height) {
            this.displayName = displayName;
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
