package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * Coral Scoring Subsystems
 */
/*
 *
 */
public class CoralScoring extends SubsystemBase {
    private CoralScoringIO io;
    private CoralScoringInputsAutoLogged coralScoringAutoLogged =
        new CoralScoringInputsAutoLogged();

    public CoralScoring(CoralScoringIO io) {
        this.io = io;
        io.updateInputs(coralScoringAutoLogged);
    }

    @Override
    public void periodic() {
        io.updateInputs(coralScoringAutoLogged);
        Logger.processInputs("Coral Scoring", coralScoringAutoLogged);
    }

    public void setScoringMotor(double percentage) {
        Logger.recordOutput("Scoring Percentage", percentage);
        io.setCoralScoringMotorPercentage(percentage);
    }

    public boolean getScoringBeamBrakeStatus() {
        return coralScoringAutoLogged.scoringBeamBrake;
    }

    /**
     * Runs Pre Scoring Motor
     */
    public Command runPreScoringMotor(double scoringSpeed) {
        return Commands.startEnd(() -> {
            setScoringMotor(scoringSpeed);
        }, () -> {
            setScoringMotor(0);
        }, this).until(() -> getScoringBeamBrakeStatus());
    }

    /**
     * Sets motor speed to score.
     */
    public Command runScoringMotor(double scoringSpeed) {
        return Commands.startEnd(() -> {
            setScoringMotor(scoringSpeed);
        }, () -> {
            setScoringMotor(0);
        }, this).until(() -> !getScoringBeamBrakeStatus());
    }


}
