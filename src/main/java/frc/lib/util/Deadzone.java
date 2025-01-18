package frc.lib.util;

import frc.robot.Constants;

/** Deadzone Utilities */
public class Deadzone {

    /** Make deadzone for an input axis, with proper scaling. */
    public static double applyDeadzone(double input) {
        return (Math.abs(input) < Constants.STICK_DEADBAND) ? 0
            : (input - Constants.STICK_DEADBAND) / (1.0 - Constants.STICK_DEADBAND);
    }

}
