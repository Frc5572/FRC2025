package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.RPM;
import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/**
 * Real Class Coral Scoring
 */

public class CoralScoringSim implements CoralScoringIO {
    private final DigitalInput outtakeBeamBreak =
        new DigitalInput(Constants.CoralScoringConstants.OUTTAKE_BEAM_BREAK_DIO_PORT);
    private final DigitalInput intakeBeamBreak =
        new DigitalInput(Constants.CoralScoringConstants.INTAKE_BEAM_BREAK_DIO_PORT);
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
        inputs.outtakeBeamBreak = !outtakeBeamBreak.get();
        inputs.intakeBeamBreak = !intakeBeamBreak.get();
        inputs.scoringRPM = RPM.of(coralMotor.getAngularVelocityRPM());
    }

    public void setCoralPower(double percent) {
        coralMotor.setInputVoltage(percent * 12.0);
    }

}
