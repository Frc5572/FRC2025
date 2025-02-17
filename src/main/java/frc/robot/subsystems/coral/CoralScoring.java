package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.viz.Viz2025;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * Coral Scoring Subsystem
 */
public class CoralScoring extends SubsystemBase {
    private CoralScoringIO io;
    private CoralScoringInputsAutoLogged inputs = new CoralScoringInputsAutoLogged();
    private final Viz2025 viz;
    public Trigger coralAtIntake = new Trigger(() -> getIntakeBeamBreakStatus());
    public Trigger coralAtOuttake = new Trigger(() -> getOuttakeBeamBreakStatus());


    private GenericEntry haveCoral =
        RobotContainer.mainDriverTab.add("Have Coral", Color.kBlack.toHexString())
            .withWidget("Single Color View").withPosition(8, 0).withSize(3, 2).getEntry();

    /**
     * Coral Scoring subsystem
     */
    public CoralScoring(CoralScoringIO io, Viz2025 viz) {
        this.viz = viz;
        this.io = io;
        io.updateInputs(inputs);
    }

    /**
     * Get outtake beambreak status
     *
     * @return Status of beambreak at outtake
     */
    public boolean getOuttakeBeamBreakStatus() {
        return inputs.outtakeBeamBreak;
    }

    /**
     * Get intake beambreak status
     *
     * @return Status of beambreak at intake
     */
    public boolean getIntakeBeamBreakStatus() {
        return inputs.intakeBeamBreak;
    }

    /**
     * Coral Motor Active. Motor if speed is greater than 2 RPM.
     *
     * @return Status of Coral Motor
     */
    public boolean getCoralMotorActiveStatus() {
        return inputs.scoringRPM.baseUnitMagnitude() > 2;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral", inputs);
        viz.setHasCoral(getOuttakeBeamBreakStatus());
        if (getIntakeBeamBreakStatus() && getOuttakeBeamBreakStatus()) {
            haveCoral.setString(Color.kBlue.toHexString());
        } else if (getIntakeBeamBreakStatus()) {
            haveCoral.setString(Color.kOrange.toHexString());
        } else if (getOuttakeBeamBreakStatus()) {
            haveCoral.setString(Color.kPurple.toHexString());
        } else {
            haveCoral.setString(Color.kBlack.toHexString());
        }
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
     *
     * @param power Power to apply to motor
     *
     * @return Command
     */
    private Command motorStartEndCommand(double power) {
        return Commands.startEnd(() -> {
            setCoralPower(power);
        }, () -> {
            setCoralPower(0);
        }, this);
    }

    /**
     * Runs Pre Scoring Motor
     *
     * @return Command
     */
    public Command runCoralIntake() {
        return motorStartEndCommand(Constants.CoralScoringConstants.INTAKE_POWER)
            .until(() -> getOuttakeBeamBreakStatus());
    }

    /**
     * Sets motor speed to score.
     *
     * @return Command
     */
    public Command runCoralOuttake() {
        return motorStartEndCommand(Constants.CoralScoringConstants.OUTTAKE_POWER).withDeadline(
            Commands.waitUntil(() -> !getOuttakeBeamBreakStatus()).andThen(Commands.waitSeconds(2)))
            .withTimeout(10);
    }
}
