package frc.robot.playbook;

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.math.AllianceFlipUtil;
import frc.lib.util.ScoringLocation;
import frc.lib.util.ScoringLocation.CoralHeight;
import frc.lib.util.ScoringLocation.CoralLocation;
import frc.robot.Constants;
import frc.robot.FieldConstants;

public final class CoralPlay implements Play {

    public final ScoringLocation.CoralLocation location;
    public final ScoringLocation.CoralHeight height;

    public CoralPlay(CoralLocation location, CoralHeight height) {
        this.location = location;
        this.height = height;
    }

    @Override
    public String getCommandString() {
        return "c" + location.toString() + height.toString();
    }

    private Command goToFeeder(PlayCommandArgs args) {
        return args.swerve().moveAndAvoidReef(
            () -> args.coralStation().get().getPose.apply(args.swerve().getPose()), true, 0.1, 1);
    }

    private Command home(PlayCommandArgs args) {
        return args.elevator().home();
    }

    private Command goToCoralScoringLocationCoarse(PlayCommandArgs args) {
        return args.swerve().moveAndAvoidReef(() -> location.targetPose, true, 0.5, 1);
    }

    private Command goToCoralScoringLocationFine(PlayCommandArgs args) {
        return args.swerve().moveToPose(() -> location.targetPose, true, 0.1, 1);
    }

    private static final double approachDistance =
        FieldConstants.Reef.circumscribedRadius.in(Meters)
            + 2.0 * Math.hypot(Constants.Swerve.bumperFront.in(Meters),
                Constants.Swerve.bumperRight.in(Meters));

    private Command approachRaiseElevator(PlayCommandArgs args) {
        return new InstantCommand().until(() -> {
            return AllianceFlipUtil.apply(args.swerve().getPose().getTranslation())
                .getDistance(FieldConstants.Reef.center) < approachDistance;
        }).andThen(args.elevator().p1());
    }

    private Command raiseElevator(PlayCommandArgs args) {
        return args.elevator().moveTo(() -> height.height.height); // lol, what is this???
    }

    private Command backAway(PlayCommandArgs args) {
        return args.swerve()
            .moveToPose(() -> Pose2d.kZero.plus(location.targetPose.minus(new Pose2d(
                new Translation2d(1.0, location.targetPose.getRotation().plus(Rotation2d.k180deg)),
                Rotation2d.kZero))), true, 0.4, 20);
    }

    @Override
    public Command asCommand(PlayCommandArgs args) {
        return goToFeeder(args).alongWith(home(args))
            .andThen(goToCoralScoringLocationCoarse(args).alongWith(approachRaiseElevator(args)))
            .andThen(goToCoralScoringLocationFine(args).alongWith(raiseElevator(args)));
    }



}
