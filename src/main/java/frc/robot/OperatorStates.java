package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * changes operator states from normal to manual
 */
public class OperatorStates {
    public Trigger manualModeCheck = new Trigger(() -> manualModeEnabled());

    public OperatorStates() {}

    public boolean manualMode = false;

    public boolean manualModeEnabled() {
        return manualMode;
    }

    public void toggleManualMode() {
        manualMode = !manualMode;
    }

    public void enableManualMode() {
        manualMode = true;
    }

    public void diableManualMode() {
        manualMode = false;
    }
}
