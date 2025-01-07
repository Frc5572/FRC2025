package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

