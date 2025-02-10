package frc.robot.playbook;

import java.util.function.Consumer;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve;

public record PlayCommandArgs(Swerve swerve, Elevator elevator, Consumer<Pose2d> setMark,
    Supplier<Pose2d> getMark, Supplier<PreferredCoralStation> coralStation,
    Supplier<PreferredDirection> direction, Trigger hasCoral, Trigger hasAlgae) {
}
