package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Scoring Locations for the 2025 game Reefscape */
public class ScoringLocation {

    /** Reef locations */
    public static enum CoralLocation {
        // @formatter:off
        A(new Pose2d(3.2095999717712402, 4.190871238708496, Rotation2d.kZero)), 
        B(new Pose2d(3.2095999717712402, 3.88173770904541, Rotation2d.kZero)), 
        C(new Pose2d(3.7248222827911377, 3.036773443222046, Rotation2d.fromDegrees(60))), 
        D(new Pose2d(4.075173377990723 , 2.810075521469116 , Rotation2d.fromDegrees(60))), 
        E(new Pose2d(4.981964588165283, 2.8512933254241943, Rotation2d.fromDegrees(120))), 
        F(new Pose2d(5.270488739013672, 2.9955556392669678, Rotation2d.fromDegrees(120))), 
        G(new Pose2d(5.765102386474609, 3.861128807067871, Rotation2d.fromDegrees(180))), 
        H(new Pose2d(5.765102386474609, 4.170262336730957, Rotation2d.fromDegrees(180))), 
        I(new Pose2d(5.270488739013672, 5.0152268409729, Rotation2d.fromDegrees(240))), 
        J(new Pose2d(4.981965065002441, 5.200706481933594, Rotation2d.fromDegrees(240))), 
        K(new Pose2d(4.013347148895264, 5.200706481933594, Rotation2d.fromDegrees(300))), 
        L(new Pose2d(3.704213857650757, 5.0358357429504395, Rotation2d.fromDegrees(300)));
        // @formatter:on

        public final Pose2d driveLocation;

        private CoralLocation(Pose2d driveLocation) {
            this.driveLocation = driveLocation;
        }

    }

    /** Reef branch heights */
    public static enum CoralHeight {
        L1, L2, L3, L4,
    }

}
