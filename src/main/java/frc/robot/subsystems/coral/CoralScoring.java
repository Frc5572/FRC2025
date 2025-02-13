package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.viz.Viz2025;
import frc.robot.RobotContainer;

/**
 * Coral Scoring Subsystem
 */
public class CoralScoring extends SubsystemBase {
    private CoralScoringIO io;
    private CoralScoringInputsAutoLogged coralScoringAutoLogged =
        new CoralScoringInputsAutoLogged();
    private final Viz2025 viz;
    public Trigger intakedCoralRight = new Trigger(() -> getIntakeBrakeStatus());
    public Trigger outtakedCoral = new Trigger(() -> getOuttakeBeamBrakeStatus());


    private GenericEntry haveCoral =
        RobotContainer.mainDriverTab.add("Is Coral State", Color.kRed.toHexString())
            .withWidget("Single Color View").withPosition(4, 2).withSize(3, 2).getEntry();

    /** Coral Scoring subsystem */
    public CoralScoring(CoralScoringIO io, Viz2025 viz) {
        this.viz = viz;
        this.io = io;
        io.updateInputs(coralScoringAutoLogged);
    }

    public boolean getOuttakeBeamBrakeStatus() {
        return coralScoringAutoLogged.scoringBeamBrake;
    }

    public boolean getIntakeBrakeStatus() {
        return coralScoringAutoLogged.grabingBeamBrakeRight;
    }

    @Override
    public void periodic() {
        io.updateInputs(coralScoringAutoLogged);
        Logger.processInputs("Coral Scoring", coralScoringAutoLogged);
        viz.setHasCoral(getScoringBeamBrakeStatus());
        if (getIntakeBrakeStatus()) {
            haveCoral.setString(Color.kGreen.toHexString());

        } else {
            haveCoral.setString(Color.kRed.toHexString());
        }
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
        return motorStartEndCommand(scoringSpeed).until(() -> getScoringBeamBrakeStatus());
    }

    /**
     * Sets motor speed to score.
     */
    public Command runScoringMotor(double scoringSpeed) {
        return motorStartEndCommand(scoringSpeed).withDeadline(
            Commands.waitUntil(() -> !getOuttakeBeamBrakeStatus()).andThen(Commands.waitSeconds(2)))
            .withTimeout(10);
    }

    // Consolidates scoring motor
    // public Command runCoralOuttakeMotor(double scoringSpeed) {
    // Command scoringMotorRun = motorStartEndCommand(scoringSpeed).withDeadline(
    // Commands.waitUntil(() -> !getScoringBeamBrakeStatus()).andThen(Commands.waitSeconds(2)))
    // .withTimeout(10);
    // Command preScoringMotorRun = motorStartEndCommand(scoringSpeed)
    // .until(() -> getScoringBeamBrakeStatus()).withTimeout(10);
    // return Commands.either(scoringMotorRun, preScoringMotorRun,
    // () -> getScoringBeamBrakeStatus());
    // }



}
