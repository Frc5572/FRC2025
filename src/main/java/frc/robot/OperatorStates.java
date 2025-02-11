package frc.robot;

public class OperatorStates {

    private OperatorStates() {}

    public static boolean manualMode = false;

    public static boolean manualModeEnabled() {
        return manualMode;
    }

    public static void toggleManualMode() {
        manualMode = !manualMode;
    }

    public static void enableManualMode() {
        manualMode = true;
    }

    public static void diableManualMode() {
        manualMode = false;
    }
}
