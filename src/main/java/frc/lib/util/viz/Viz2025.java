package frc.lib.util.viz;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Visualization of the 2025 Robot */
public class Viz2025 implements Drawable {

    private final FieldViz fieldViz;
    private final String prefix;
    private double elevatorHeight = 0.0;
    private double algaeAngle = 0.0;
    private double algaeAngularVelocity = 0.0;
    private boolean algaeDropped = false;
    private Pose2d estimatedPose;
    private Pose2d actualPose = new Pose2d();
    private final Pose3d[] mechanisms = new Pose3d[11];
    private boolean hasCoral = true;
    private boolean hasAlgae = true;
    private static final Transform3d coralOffset = new Transform3d(
        new Translation3d(0.236157, -0.157444, 0.383131), new Rotation3d(0, Math.toRadians(42), 0));
    private static final Transform3d algaeOffset =
        new Transform3d(new Translation3d(0.389977, 0, 0.602934), Rotation3d.kZero);

    private static final int CLIMBER_ID = 0;
    private static final int STAGE2_ID = 1;
    private static final int STAGE3_ID = 2;
    private static final int STAGE4_ID = 3;
    private static final int ALGAE_ID = 4;
    private static final int BL_ID = 5;
    private static final int BR_ID = 6;
    private static final int FL_ID = 7;
    private static final int FR_ID = 8;
    private static final int RED_BUMPER = 9;
    private static final int BLUE_BUMPER = 10;

    private final Pose3d useBumper = new Pose3d(new Translation3d(0, 0, 1), Rotation3d.kZero);
    private final Pose3d invisible = new Pose3d(new Translation3d(0, 0, -100), Rotation3d.kZero);

    /** Visualization of the 2025 Robot */
    public Viz2025(FieldViz fieldViz, String prefix) {
        this.fieldViz = fieldViz;
        this.prefix = prefix;
        for (int i = 0; i < mechanisms.length; i++) {
            mechanisms[i] = new Pose3d();
        }
        this.reset(Pose2d.kZero);
        // SmartDashboard.setDefaultNumber("ClimberX", 0.505);
        SmartDashboard.setDefaultNumber("ClimberZ", -0.055);
    }

    /** Set the angle of the climber, with 0 being straight up and down. */
    public void setClimberAngle(Angle angle) {
        Angle visAngle = angle.div(300);
        SmartDashboard.putNumber("Real Angle", angle.in(Degrees));
        // SmartDashboard.putNumber("ClimberX", -visAngle.in(Radians) * 0.531);
        SmartDashboard.putNumber("ASDFDSF", visAngle.in(Radians));
        // SmartDashboard.putNumber("SIN", Math.sin(visAngle.in(Radians)));
        Translation3d trans = new Translation3d(-visAngle.in(Radians) * 0.505, 0,
            -SmartDashboard.getNumber("ClimberZ", 0) * visAngle.in(Radians));
        mechanisms[CLIMBER_ID] =
            new Pose3d(trans, new Rotation3d(Radians.of(0.0), visAngle, Radians.of(0.0)));
    }

    /** Show coral in the coral scorer. */
    public void setHasCoral(boolean hasCoral) {
        this.hasCoral = hasCoral;
    }

    /** Show algae in the algae scorer. */
    public void setHasAlgae(boolean hasAlgae) {
        this.hasAlgae = hasAlgae;
    }

    /** Set the estimated drivetrain state, including swerve drive poses.` */
    public void setDrivetrainState(Pose2d estimatedPose, Rotation2d[] states) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                mechanisms[RED_BUMPER] = useBumper;
                mechanisms[BLUE_BUMPER] = Pose3d.kZero;
            } else {
                mechanisms[BLUE_BUMPER] = useBumper;
                mechanisms[RED_BUMPER] = Pose3d.kZero;
            }
        }
        this.estimatedPose = estimatedPose;
        if (RobotBase.isReal()) {
            this.actualPose = estimatedPose;
        }
        mechanisms[BL_ID] = new Pose3d(new Translation3d(Constants.Swerve.moduleTranslations[0]),
            new Rotation3d(states[0]));
        mechanisms[BR_ID] = new Pose3d(new Translation3d(Constants.Swerve.moduleTranslations[1]),
            new Rotation3d(states[1]));
        mechanisms[FL_ID] = new Pose3d(new Translation3d(Constants.Swerve.moduleTranslations[2]),
            new Rotation3d(states[2]));
        mechanisms[FR_ID] = new Pose3d(new Translation3d(Constants.Swerve.moduleTranslations[3]),
            new Rotation3d(states[3]));
    }

    /** Set the actual pose (only different for sim) */
    public void setActualPose(Pose2d pose) {
        this.actualPose = pose;
    }

    /** Set the elevator height relative to its lowest point. */
    public void setElevatorHeight(Distance height) {
        elevatorHeight = height.in(Meters);
    }

    /** Reset all values, getting ready for auto. */
    public void reset(Pose2d pose) {
        elevatorHeight = 0.0;
        algaeAngle = 0.0;
        algaeDropped = false;
        estimatedPose = pose;
        actualPose = pose;
        mechanisms[CLIMBER_ID] = new Pose3d(Translation3d.kZero, Rotation3d.kZero);
    }

    /** Publish all values to Logger */
    @Override
    public void drawImpl() {
        algaeAngle = 90.0;
        mechanisms[STAGE2_ID] =
            new Pose3d(new Translation3d(0, 0, Math.min(elevatorHeight, 0.862013 - 0.173028)),
                Rotation3d.kZero);
        mechanisms[STAGE3_ID] =
            new Pose3d(new Translation3d(0, 0, Math.min(elevatorHeight, 1.57956 - 0.199172)),
                Rotation3d.kZero);
        mechanisms[STAGE4_ID] =
            new Pose3d(new Translation3d(0, 0, Math.min(elevatorHeight, 1.96337 - 0.226559)),
                Rotation3d.kZero);
        Rotation3d algaeRotation3d = new Rotation3d(0.0, Units.degreesToRadians(algaeAngle), 0.0);
        Translation3d algaeTranslation3d = new Translation3d(0.13335, 0.006358,
            0.615387 + Math.min(elevatorHeight, 1.96337 - 0.226559));
        mechanisms[ALGAE_ID] = new Pose3d(algaeTranslation3d, algaeRotation3d);
        Logger.recordOutput(prefix + "Viz/Mechanisms", mechanisms);
        Logger.recordOutput(prefix + "Viz/EstimatedPose", estimatedPose);
        Logger.recordOutput(prefix + "Viz/ActualPose", actualPose);
        if (hasCoral) {
            Logger.recordOutput(prefix + "Viz/HeldCoral", new Pose3d(actualPose)
                .plus(mechanisms[STAGE4_ID].minus(Pose3d.kZero)).plus(coralOffset));
        } else {
            Logger.recordOutput(prefix + "Viz/HeldCoral", invisible);
        }
        if (hasAlgae) {
            Logger.recordOutput(prefix + "Viz/HeldAlgae", new Pose3d(actualPose)
                .plus(mechanisms[STAGE4_ID].minus(Pose3d.kZero)).plus(algaeOffset));
        } else {
            Logger.recordOutput(prefix + "Viz/HeldAlgae", invisible);
        }
    }

}
