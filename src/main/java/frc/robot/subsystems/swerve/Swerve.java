package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveInputsAutoLogged;

/**
 * Drivetrain subsystem.
 */

public class Swerve extends SubsystemBase {
    private SwerveIO io;
    private SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

    /**
     * Create Wrist Intake Subsystem
     */
    public Swerve(SwerveIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drivetrain", inputs);
    }
}

