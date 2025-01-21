package frc.robot.subsystems.climber;


import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Climber Subsystem
 */
public class Climber extends SubsystemBase {
    private ClimberIO io;
    private ClimberInputsAutoLogged climberAutoLogged = new ClimberInputsAutoLogged();

    // private GenericEntry beamBrake = RobotContainer.mainDriverTab.add("Have Note", false)
    // .withWidget(BuiltInWidgets.kBooleanBox).withPosition(9, 4).withSize(3, 2).getEntry();

    // private String noNote = Color.kBlack.toHexString();
    // private GenericEntry haveNote = RobotContainer.mainDriverTab.add("Have Note", noNote)
    // .withWidget("Single Color View").withPosition(9, 4).withSize(3, 2).getEntry();

    /**
     * Climber Subsystem
     *
     * @param io IO Layer
     * @param viz Sim Visualization
     */
    public Climber(ClimberIO io) {
        this.io = io;
        // this.viz = viz;
        io.updateInputs(climberAutoLogged);
    }

    @Override
    public void periodic() {
        io.updateInputs(climberAutoLogged);
        Logger.processInputs("Climber", climberAutoLogged);

    }

    /**
     * Set the power of both Climber motors
     *
     * @param percentage 0-1 power for the Climber motors
     */
    public void setClimberMotor(double percentage) {
        Logger.recordOutput("/Climber/Climber Percentage", percentage);
        io.setClimberMotorPercentage(percentage);
    }

    /**
     * Set the power for the indexer motor
     *
     * @param percentage 0-1 power for the indexer motor
     */
    public void setIndexerMotor(double percentage) {
        Logger.recordOutput("/Climber/Indexer Percentage", percentage);
        io.setIndexerMotorPercentage(percentage);
    }

    /**
     * Get the status of the indexer beam brake.
     *
     * @return True if beam brake is broken, False if open
     */
    public boolean getIndexerBeamBrakeStatus() {
        return climberAutoLogged.indexerBeamBrake;
    }

    /**
     * Get the status of the Climber beam brake.
     *
     * @return True if beam brake is broken, False if open
     */
    public boolean getClimberBeamBrakeStatus() {
        return climberAutoLogged.climberBeamBrake;
    }

    /**
     * Command to run the Climber motor and indexer until the sensor trips
     * 
     * @return {@link Command} to run the Climber and indexer motors
     */
    public Command runClimberMotor(double climberSpeed, double indexerSpeed) {
        return Commands.startEnd(() -> {
            setClimberMotor(climberSpeed);
            setIndexerMotor(indexerSpeed);
        }, () -> {
            setClimberMotor(0);
            setIndexerMotor(0);
        }, this).until(() -> getIndexerBeamBrakeStatus()).unless(() -> getIndexerBeamBrakeStatus());
    }

    /**
     * Command to run the Climber motor and indexer until the sensor trips
     *
     * @return {@link Command} to run the Climber and indexer motors
     */
    public Command runClimberMotorNonStop(double climberSpeed, double indexerSpeed) {
        return Commands.startEnd(() -> {
            setClimberMotor(climberSpeed);
            setIndexerMotor(indexerSpeed);
        }, () -> {
            setClimberMotor(0);
            setIndexerMotor(0);
        }, this);
    }

    /**
     * Command to run the indexer
     *
     * @return {@link Command} to run the indexer motors
     */
    public Command runIndexerMotor(double speed) {
        return Commands.startEnd(() -> {
            setIndexerMotor(speed);
        }, () -> {
            setIndexerMotor(0);
        }, this);
    }
}
