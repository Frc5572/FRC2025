package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.RPM;
import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Real Class Coral Scoring
 */

public class CoralScoringSim implements CoralScoringIO {
    private DCMotorSim coralMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(.5, .5),
        DCMotor.getNeo550(1).withReduction(4));

    /**
     * Coral Scoring Real
     */
    public CoralScoringSim() {}

    /**
     * updating coral beam brakes
     */

    public void updateInputs(CoralScoringInputs inputs) {
        coralMotor.update(LoggedRobot.defaultPeriodSecs);
        // For sim, just assume we have coral all the time.
        inputs.outtakeBeamBreak = true;
        inputs.intakeBeamBreak = true;
        inputs.scoringRPM = RPM.of(coralMotor.getAngularVelocityRPM());
    }

    public void setCoralPower(double percent) {
        coralMotor.setInputVoltage(percent * 12.0);
    }

}
