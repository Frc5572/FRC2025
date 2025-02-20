package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.viz.Viz2025;

/**
 * Coral Scoring Subsystem
 */
public class CoralScoring extends SubsystemBase {
    private CoralScoringIO io;
    private CoralScoringInputsAutoLogged coralScoringAutoLogged =
        new CoralScoringInputsAutoLogged();
    private final Viz2025 viz;
    public Trigger coralAtIntake = new Trigger(() -> getIntakeBeamBreakStatus()).debounce(.25);
    public Trigger coralAtOuttake = new Trigger(() -> getOuttakeBeamBreakStatus()).debounce(.25);

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
        return coralScoringAutoLogged.intakeBeamBrake;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral", inputs);
        viz.setHasCoral(getOuttakeBeamBreakStatus());
        Color temp = Color.kBlack;
        if (getIntakeBeamBreakStatus() && getOuttakeBeamBreakStatus()) {
            temp = Color.kBlue;
        } else if (getIntakeBeamBreakStatus()) {
            temp = Color.kOrange;
        } else if (getOuttakeBeamBreakStatus()) {
            temp = Color.kPurple;
        }
        SmartDashboard.putString("Dashboard/Main Driver/Have Coral", temp.toHexString());
    }

    /**
     * Set motor power
     *
     * @param power power to apply to motor
     */
    public void setCoralPower(double power) {
        Logger.recordOutput("Coral/Power", power);
        io.setCoralPower(power);
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
        return motorStartEndCommand(scoringSpeed).until(() -> getOuttakeBeamBrakeStatus());
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
