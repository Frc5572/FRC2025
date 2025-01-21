
package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meters;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.draw.DrawingUtils;
import frc.lib.draw.Line;
import frc.lib.math.AllianceFlipUtil;
import frc.lib.math.Circle;
import frc.lib.math.FieldConstants;
import frc.lib.util.swerve.SwerveModule;
import frc.robot.Constants;
import frc.robot.Constants.ReefNavigation.FeederStation;
import frc.robot.Constants.ReefNavigation.ReefBranch;

/**
 * Swerve Subsystem
 */
public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] swerveMods;
    private final Field2d field = new Field2d();
    private double fieldOffset;
    private SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();
    private SwerveIO swerveIO;

    /**
     * Swerve Subsystem
     */
    public Swerve(SwerveIO swerveIO) {
        this.swerveIO = swerveIO;
        swerveMods = swerveIO.createModules();
        fieldOffset = getGyroYaw().getDegrees();

        swerveOdometry = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics,
            getGyroYaw(), getModulePositions(), new Pose2d());

        swerveIO.updateInputs(inputs);
    }

    /**
     * Tele-Op Drive method
     *
     * @param translation The magnitude in XY
     * @param rotation The magnitude in rotation
     * @param fieldRelative Whether or not field relative
     * @param isOpenLoop Whether or not Open or Closed Loop
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative,
        boolean isOpenLoop) {
        ChassisSpeeds chassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(),
                rotation, getFieldRelativeHeading())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        setModuleStates(chassisSpeeds);
    }

    /**
     * Set Swerve Module States
     *
     * @param desiredStates Array of desired states
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        Logger.recordOutput("/Swerve/DesiredStates", desiredStates);
        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /**
     * Sets swerve module states using Chassis Speeds.
     *
     * @param chassisSpeeds The desired Chassis Speeds
     */
    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(swerveModuleStates);
    }

    /**
     * Get current Chassis Speeds
     *
     * @return The current {@link ChassisSpeeds}
     */
    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Get Swerve Module States
     *
     * @return Array of Swerve Module States
     */
    @AutoLogOutput(key = "Swerve/ModuleStates")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * Get Swerve Module Positions
     *
     * @return Array of Swerve Module Positions
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Get Position on field from Odometry
     *
     * @return Pose2d on the field
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    /**
     * Set the position on the field with given Pose2d
     *
     * @param pose Pose2d to set
     */
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        this.swerveIO.setPose(pose);
    }

    /**
     * Get Rotation of robot from odometry
     *
     * @return Heading of robot relative to the field as {@link Rotation2d}
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Get Rotation from the gyro
     *
     * @return Current rotation/yaw of gyro as {@link Rotation2d}
     */
    public Rotation2d getGyroYaw() {
        double yaw = inputs.yaw;
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromRotations(-yaw)
            : Rotation2d.fromRotations(yaw);
    }

    /**
     * Get Field Relative Heading
     *
     * @return The current field relative heading in {@link Rotation2d}
     */
    public Rotation2d getFieldRelativeHeading() {
        return Rotation2d.fromDegrees(getGyroYaw().getDegrees() - fieldOffset);
    }

    /**
     * Resets the gyro field relative driving offset
     */
    public void resetFieldRelativeOffset() {
        fieldOffset = getGyroYaw().getDegrees() + 180;
    }

    @Override
    public void periodic() {

        swerveIO.updateInputs(inputs);
        for (var mod : swerveMods) {
            mod.periodic();
        }
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        Logger.processInputs("Swerve", inputs);



    }

    /**
     * Sets motors to 0 or inactive.
     */
    public void setMotorsZero() {
        System.out.println("Setting Zero!!!!!!");
        setModuleStates(new ChassisSpeeds(0, 0, 0));
    }

    /**
     * Make an X pattern with the wheels
     */
    public void wheelsIn() {
        swerveMods[0].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(45)), false);
        swerveMods[1].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(135)), false);
        swerveMods[2].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-45)), false);
        swerveMods[3].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-135)),
            false);
        this.setMotorsZero();
    }

    /**
     * Gets a list containing all 4 swerve module positions
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Determine whether or not to flight the auto path
     *
     * @return True if flip path to Red Alliance, False if Blue
     */
    public static boolean shouldFlipPath() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            return ally.get() == Alliance.Red;
        }
        return false;
    }

    /**
     * Creates a command for driving the swerve drive during tele-op
     *
     * @param controller The driver controller
     * @param fieldRelative Whether the movement is relative to the field or absolute
     * @param openLoop Open or closed loop system
     */
    public Command teleOpDrive(CommandXboxController controller, boolean fieldRelative,
        boolean openLoop) {
        return this.run(() -> {
            double speedMultiplier = 1;
            double yaxis = -controller.getLeftY() * speedMultiplier;
            double xaxis = -controller.getLeftX() * speedMultiplier;
            double raxis = -controller.getRightX() * speedMultiplier;
            /* Deadbands */
            yaxis = (Math.abs(yaxis) < Constants.STICK_DEADBAND) ? 0
                : (yaxis - Constants.STICK_DEADBAND) / (1.0 - Constants.STICK_DEADBAND);
            xaxis = (Math.abs(xaxis) < Constants.STICK_DEADBAND) ? 0
                : (xaxis - Constants.STICK_DEADBAND) / (1.0 - Constants.STICK_DEADBAND);
            xaxis *= xaxis * Math.signum(xaxis);
            yaxis *= yaxis * Math.signum(yaxis);
            raxis = (Math.abs(raxis) < Constants.STICK_DEADBAND) ? 0 : raxis;
            Translation2d translation =
                new Translation2d(yaxis, xaxis).times(Constants.Swerve.maxSpeed);
            double rotation = raxis * Constants.Swerve.maxAngularVelocity;
            this.drive(translation, rotation, fieldRelative, openLoop);
        });
    }

    /** Move to a given {@link Pose2d}. */
    public Command moveToPose(Supplier<Pose2d> pose2dSupplier, boolean flipForRed, double tol) {
        Swerve swerveCopy = this;

        HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
            new PIDController(Constants.SwerveTransformPID.PID_XKP,
                Constants.SwerveTransformPID.PID_XKI, Constants.SwerveTransformPID.PID_XKD),
            new PIDController(Constants.SwerveTransformPID.PID_YKP,
                Constants.SwerveTransformPID.PID_YKI, Constants.SwerveTransformPID.PID_YKD),
            new ProfiledPIDController(Constants.SwerveTransformPID.PID_TKP,
                Constants.SwerveTransformPID.PID_TKI, Constants.SwerveTransformPID.PID_TKD,
                new TrapezoidProfile.Constraints(Constants.SwerveTransformPID.MAX_ANGULAR_VELOCITY,
                    Constants.SwerveTransformPID.MAX_ANGULAR_ACCELERATION)));
        holonomicDriveController.setTolerance(new Pose2d(tol, tol, Rotation2d.fromDegrees(1)));
        Command c = new Command() {

            private Pose2d pose2d;
            private Swerve swerve = swerveCopy;

            @Override
            public void initialize() {
                pose2d = pose2dSupplier.get();
                if (flipForRed) {
                    pose2d = AllianceFlipUtil.apply(pose2d);
                }
            }

            @Override
            public void execute() {
                ChassisSpeeds ctrlEffort = holonomicDriveController.calculate(swerve.getPose(),
                    pose2d, 0, pose2d.getRotation());
                swerve.setModuleStates(ctrlEffort);
            }

            @Override
            public void end(boolean interrupted) {
                swerve.setMotorsZero();
            }

            @Override
            public boolean isFinished() {
                return holonomicDriveController.atReference();
            }
        };
        c.addRequirements(this);
        return c;
    }

    private static class MoveAndAvoidReef extends Command {

        private final Swerve swerve;
        private final HolonomicDriveController holonomicDriveController =
            new HolonomicDriveController(
                new PIDController(Constants.SwerveTransformPID.PID_XKP,
                    Constants.SwerveTransformPID.PID_XKI, Constants.SwerveTransformPID.PID_XKD),
                new PIDController(Constants.SwerveTransformPID.PID_YKP,
                    Constants.SwerveTransformPID.PID_YKI, Constants.SwerveTransformPID.PID_YKD),
                new ProfiledPIDController(Constants.SwerveTransformPID.PID_TKP,
                    Constants.SwerveTransformPID.PID_TKI, Constants.SwerveTransformPID.PID_TKD,
                    new TrapezoidProfile.Constraints(
                        Constants.SwerveTransformPID.MAX_ANGULAR_VELOCITY,
                        Constants.SwerveTransformPID.MAX_ANGULAR_ACCELERATION)));
        private Pose2d pose2d;
        private Rotation2d targetAngle;
        private final Supplier<Pose2d> pose2dSupplier;

        private static final Line[] lines;
        private static final Circle circle = new Circle("reef/circle", FieldConstants.Reef.center,
            Constants.ReefNavigation.reefNavigateAroundCircleRadius, "#00ff00");
        private static final Circle innerCircle = new Circle("reef/innerCircle",
            FieldConstants.Reef.center, Constants.ReefNavigation.reefAvoidCircleRadius, "#ff0000");

        static {
            lines = new Line[3];
            for (int i = 0; i < 3; i++) {
                lines[i] =
                    new Line("reef/line" + i, new Translation2d(), new Translation2d(), "#0000ff");
                DrawingUtils.addDrawable(lines[i]);
            }
            DrawingUtils.addDrawable(circle);
            DrawingUtils.addDrawable(innerCircle);
        }

        public MoveAndAvoidReef(Swerve swerve, Supplier<Pose2d> pose2dSupplier, double tol) {
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            holonomicDriveController.setTolerance(new Pose2d(tol, tol, Rotation2d.fromDegrees(2)));
            holonomicDriveController.getXController().setIZone(1.0);
            holonomicDriveController.getYController().setIZone(1.0);
            holonomicDriveController.getThetaController().setIZone(1.0);
            super.addRequirements(swerve);
        }

        private static boolean hasFlipped = false;

        @Override
        public void initialize() {
            pose2d = pose2dSupplier.get();
            pose2d = AllianceFlipUtil.apply(pose2d);
            Logger.recordOutput("reef/target", pose2d);
            if (!hasFlipped) {
                circle.center = AllianceFlipUtil.apply(circle.center);
                innerCircle.center = AllianceFlipUtil.apply(innerCircle.center);
            }
            hasFlipped = true;
            targetAngle = pose2d.getTranslation().minus(circle.center).getAngle();
            Logger.recordOutput("reef/targetAngle", targetAngle);
        }

        @Override
        public void execute() {
            Pose2d curPose = swerve.getPose();
            lines[0].start = curPose.getTranslation();
            lines[0].end = pose2d.getTranslation();
            Pose2d targetPose;
            Translation2d vec = curPose.getTranslation().minus(innerCircle.center);
            Logger.recordOutput("reef/vec", vec);
            Rotation2d curAngle = vec.getAngle();
            Logger.recordOutput("reef/curAngle", curAngle);
            Rotation2d angleDiff = targetAngle.minus(curAngle);
            Logger.recordOutput("reef/angleDiff", angleDiff);
            if (innerCircle.intersectsLine(curPose.getTranslation(), pose2d.getTranslation())
                && Math.abs(angleDiff.getDegrees()) > 5.0) {
                double dist = vec.getNorm();
                if (dist < circle.radius.in(Meters)
                    + Constants.ReefNavigation.reefNavigateAroundCircleMargin.in(Meters)) {
                    Logger.recordOutput("reef/avoidState", "In Circle");
                    while (angleDiff.getRadians() > Rotation2d.k180deg.getRadians()) {
                        angleDiff = angleDiff.minus(Rotation2d.k180deg);
                    }
                    while (angleDiff.getRadians() < Rotation2d.k180deg.unaryMinus().getRadians()) {
                        angleDiff = angleDiff.plus(Rotation2d.k180deg);
                    }
                    Rotation2d currentTargetAngle =
                        curAngle.plus(Rotation2d.fromRadians(Math.signum(angleDiff.getRadians())
                            * Math.min(Constants.ReefNavigation.reefNavigateAroundCircleResolution
                                .getRadians(), Math.abs(angleDiff.getRadians()))));

                    targetPose = new Pose2d(
                        circle.center
                            .plus(new Translation2d(circle.radius.in(Meters), currentTargetAngle)),
                        pose2d.getRotation());

                    Logger.recordOutput("reef/angleDiff", angleDiff);
                    Logger.recordOutput("reef/currentTargetAngle", currentTargetAngle);
                } else {
                    Logger.recordOutput("reef/avoidState", "Out of Circle");
                    var points = circle.circleTangentPoints(curPose.getTranslation()).get();
                    lines[1].start = curPose.getTranslation();
                    lines[2].start = curPose.getTranslation();
                    lines[1].end = points.getFirst().getSecond();
                    lines[2].end = points.getSecond().getSecond();
                    double h1 = heuristic(curPose.getTranslation(), points.getFirst(), "reef/h1");
                    double h2 = heuristic(curPose.getTranslation(), points.getSecond(), "reef/h2");
                    Logger.recordOutput("reef/h1/value", h1);
                    Logger.recordOutput("reef/h2/value", h2);
                    if (h1 < h2) {
                        targetPose =
                            new Pose2d(points.getFirst().getSecond(), pose2d.getRotation());
                    } else {
                        targetPose =
                            new Pose2d(points.getSecond().getSecond(), pose2d.getRotation());
                    }
                }
            } else {
                Logger.recordOutput("reef/avoidState", "Straight Ahead");
                targetPose = pose2d;
            }
            Logger.recordOutput("reef/intermediateTarget", targetPose);
            ChassisSpeeds ctrlEffort = holonomicDriveController.calculate(swerve.getPose(),
                targetPose, 0, pose2d.getRotation());
            swerve.setModuleStates(ctrlEffort);
        }

        private double heuristic(Translation2d f, Pair<Rotation2d, Translation2d> p,
            String prefix) {
            double r = circle.radius.in(Meters);
            Rotation2d angleDiff = targetAngle.minus(p.getFirst());
            double circum = angleDiff.getRadians() * r;
            double dist = f.getDistance(p.getSecond());
            Logger.recordOutput(prefix + "/angleDiff", angleDiff);
            Logger.recordOutput(prefix + "/circum", circum);
            Logger.recordOutput(prefix + "/dist", dist);
            return dist + circum;
        }

        @Override
        public void end(boolean interrupted) {
            // Logger.recordOutput("reef/avoidState", "Done");
            swerve.setMotorsZero();
        }

        @Override
        public boolean isFinished() {
            return holonomicDriveController.atReference();
        }

    }

    /** `moveToPose` but avoid hitting the reef. */
    public Command moveAndAvoidReef(Supplier<Pose2d> pose2dSupplier, double tol) {
        return new MoveAndAvoidReef(this, pose2dSupplier, tol);
    }

    private static Translation2d feederStationBack =
        new Translation2d(0.5716620683670044, 1.3468445539474487);
    private static Translation2d feederStationFront =
        new Translation2d(1.643324375152588, 0.5843156576156616);
    private static Rotation2d feederStationRightAngle = Rotation2d.fromRadians(-0.6350267301978877);
    private static Rotation2d feederStationLeftAngle = Rotation2d.fromRadians(0.6350267301978877);

    /**
     * Go to a given feeder station. If forward amount is 1.0, go to the furthest from the driver
     * station point possible while still being able to feed. If the forward amount is 0.0, go to
     * the closest point on the feeder station to the driver stations.
     */
    public Command feederStation(FeederStation feederStation, double forwardAmount) {
        Translation2d translation = feederStationBack
            .plus(feederStationFront.minus(feederStationBack).times(forwardAmount));
        Rotation2d rotation = feederStationRightAngle;
        if (feederStation == FeederStation.Left) {
            translation = new Translation2d(translation.getX(),
                FieldConstants.fieldWidth - translation.getY());
            rotation = feederStationLeftAngle;
        }
        Pose2d target = new Pose2d(translation, rotation);
        return moveAndAvoidReef(() -> target, 0.3);
    }

    /** Go to a given Reef Branch. */
    public Command reef(ReefBranch branch) {
        return moveAndAvoidReef(() -> branch.safePose, 0.3)
            .andThen(moveToPose(() -> branch.pose, true, 0.05))
            .andThen(moveToPose(() -> branch.safePose, true, 0.05));
    }
}
