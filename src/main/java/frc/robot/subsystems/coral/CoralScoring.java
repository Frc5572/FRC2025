package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Coral Scoring Subsystems
 */
public class CoralScoring extends SubsystemBase {
    private CoralScoringIO io;
    private CoralScoringInputsAutoLogged coralScoringAutoLogged =
        new CoralScoringInputsAutoLogged();
    private Color grabbedCoral = Color.kRed;
    private Color readyToOuttakeCoral = Color.kWhite;
    private Color coralOuttaked = Color.kYellow;

    public CoralScoring(CoralScoringIO io) {
        this.io = io;
        io.updateInputs(coralScoringAutoLogged);
    }

    public boolean getScoringBeamBrakeStatus() {
        return coralScoringAutoLogged.scoringBeamBrake;
    }

    public boolean getGrabingBeamBrakeStatus() {
        return coralScoringAutoLogged.grabingBeamBrake;
    }

    @Override
    public void periodic() {
        io.updateInputs(coralScoringAutoLogged);
        Logger.processInputs("Coral Scoring", coralScoringAutoLogged);
        // if (getScoringBeamBrakeStatus() == true){
        // Commands.setLEDsSolid(Color coralOuttaked);
        // }
    }

    public void setScoringMotor(double percentage) {
        Logger.recordOutput("Scoring Percentage", percentage);
        io.setCoralScoringMotorPercentage(percentage);
    }

    /**
     * Command that returns a command
     */

    private Command motorStartEndCommand(double scoringSpeed) {
        return Commands.startEnd(() -> {
            setScoringMotor(scoringSpeed);
        }, () -> {
            setScoringMotor(0);
        }, this);
    }

    /**
     * Runs Pre Scoring Motor
     */
    public Command runPreScoringMotor(double scoringSpeed) {
        return motorStartEndCommand(scoringSpeed).until(() -> getScoringBeamBrakeStatus())
            .withTimeout(10);
    }

    /**
     * Sets motor speed to score.
     */
    public Command runScoringMotor(double scoringSpeed) {
        return runScoringMotor(scoringSpeed).withDeadline(
            Commands.waitUntil(() -> !getScoringBeamBrakeStatus()).andThen(Commands.waitSeconds(2)))
            .withTimeout(10);

    }



}
