package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/** Utilities for flipping based on alliance */
public class AllianceFlipUtil {
    public static double fieldWidth = Units.feetToMeters(26.0) + Units.inchesToMeters(5.0);
    public static double fieldLength = Units.feetToMeters(57.0) + Units.inchesToMeters(6.875);

    /** Possibly flip */
    public static int applyAprilTag(int fiducialId) {
        if (shouldFlip()) {
            switch (fiducialId) {
                // Barge
                case 14:
                    return 5;
                case 15:
                    return 4;
                // Processor
                case 16:
                    return 3;
                // Feeder Stations
                case 13:
                    return 1;
                case 12:
                    return 2;
                // Reef
                case 18:
                    return 7;
                case 17:
                    return 8;
                case 22:
                    return 9;
                case 21:
                    return 10;
                case 20:
                    return 11;
                case 19:
                    return 6;
                default:
                    return fiducialId;
            }
        } else {
            return fiducialId;
        }
    }

    /** Possibly flip */
    public static double applyX(double x) {
        return shouldFlip() ? fieldLength - x : x;
    }

    /** Possibly flip */
    public static double applyY(double y) {
        return shouldFlip() ? fieldWidth - y : y;
    }

    /** Possibly flip */
    public static Translation2d apply(Translation2d translation) {
        return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
    }

    /** Possibly flip */
    public static Rotation2d apply(Rotation2d rotation) {
        return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
    }

    /** Possibly flip */
    public static Pose2d apply(Pose2d pose) {
        return shouldFlip() ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
            : pose;
    }

    /** Possibly flip */
    public static boolean shouldFlip() {
        return DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }
}
