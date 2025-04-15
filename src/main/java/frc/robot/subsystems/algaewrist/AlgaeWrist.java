package frc.robot.subsystems.algaewrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.viz.Viz2025;
import frc.robot.Constants;

/** Algae Wrist */
public class AlgaeWrist extends SubsystemBase {

    private final Viz2025 viz;
    private final AlgaeWristIO io;
    private final AlgaeWristInputsAutoLogged inputs = new AlgaeWristInputsAutoLogged();

    /** Algae Wrist */
    public AlgaeWrist(Viz2025 viz, AlgaeWristIO io) {
        this.viz = viz;
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeWrist", inputs);

        viz.setAlgaeAngle(inputs.wristAngle);
    }

    /** set wrist voltage */
    public Command runVolts(DoubleSupplier volts) {
        return this.runOnce(() -> io.setWristVoltage(volts.getAsDouble()));
    }

    /** Get within 5 degrees of a desired angle */
    public Command goToAngle(Supplier<Angle> angle) {
        return this.runOnce(() -> io.setWristSetpoint(angle.get())).andThen(Commands.waitUntil(
            () -> Math.abs(inputs.wristAngle.in(Rotations) - angle.get().in(Rotations)) < Degrees
                .of(5).in(Rotations)));
    }

    /** Continuously set the angle of the algae intake */
    public Command followAngle(Supplier<Angle> angle) {
        return this.run(() -> {
            io.setWristSetpoint(angle.get());
        });
    }

    /** Angle for barge */
    public Command bargeAngle() {
        return goToAngle(() -> Constants.Algae.BARGE_ANGLE);
    }

    /** Angle for home */
    public Command homeAngle() {
        return goToAngle(() -> Constants.Algae.HOME_ANGLE);
    }

    /** Angle for ground intake */
    public Command groundAngle() {
        return goToAngle(() -> Constants.Algae.GROUND_ANGLE);
    }

    /** Angle for picking off of reef */
    public Command reefAngle() {
        return goToAngle(() -> Constants.Algae.REEF_ANGLE);
    }

    /** Make wrist backdrivable */
    public Command coast() {
        return Commands.runEnd(() -> io.setBrakeMode(false), () -> io.setBrakeMode(true), this);
    }
}
