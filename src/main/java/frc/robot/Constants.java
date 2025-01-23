package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

/**
 * Constants file.
 */
public final class Constants {
    /**
     * Stick Deadband
     */
    public static final double STICK_DEADBAND = 0.1;
    /**
     * Driver ID
     */
    public static final int driverId = 0;
    /**
     * Operator ID
     */
    public static final int operatorId = 1;

    public static final boolean tuningMode = false;


    /**
     * Motor CAN id's.
     */
    public static final class Motors {
    }



    /**
     * Swerve Constants
     */
    public static final class Swerve {
        public static final double AUTO_ROTATION_KP = 5.0;
        public static final double AUTO_ROTATION_KI = 0.0;
        public static final double AUTO_ROTATION_KD = 0.0;

        public static final NavXComType navXID = NavXComType.kMXP_SPI;
        public static final boolean invertGyro = false;
        public static final boolean isFieldRelative = true;
        public static final boolean isOpenLoop = false;

        /* Drivetrain Constants */
        public static final Distance trackWidth = Inches.of(24.229);
        public static final Distance wheelBase = Inches.of(24.229);
        public static final Distance wheelDiameter = Inches.of(3.8);
        public static final Distance wheelCircumference = wheelDiameter.times(Math.PI);

        /** Get translations of each module. */
        public static Translation2d[] getModuleTranslations() {
            return new Translation2d[] {
                new Translation2d(wheelBase.in(Meters) / 2, trackWidth.in(Meters) / 2),
                new Translation2d(wheelBase.in(Meters) / 2, -trackWidth.in(Meters) / 2),
                new Translation2d(-wheelBase.in(Meters) / 2, trackWidth.in(Meters) / 2),
                new Translation2d(-wheelBase.in(Meters) / 2, -trackWidth.in(Meters) / 2)};
        }

        /*
         * Swerve Kinematics No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(getModuleTranslations());

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        public static final InvertedValue driveMotorInvert =
            InvertedValue.CounterClockwise_Positive;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert =
            SensorDirectionValue.CounterClockwise_Positive;

        /* Swerve Current Limiting */


        /*
         * These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.0;
        public static final double AUTO_MAX_SPEED = 3.0;
        /** Radians per Second */
        public static final double maxAngularVelocity = 6.28;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */

        /**
         * Front Left Module - Module 0
         */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.008789);

        }

        /**
         * Front Right Module - Module 1
         */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.298096);

        }

        /**
         * Back Left Module - Module 2
         */
        public static final class Mod2 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.451172);

        }

        /**
         * Back Right Module - Module 3
         */
        public static final class Mod3 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.321777);
        }

        /** Swerve Module constants shared across all modules. */
        public static class ModuleConstants {

            public static final DCMotor driveMotor = DCMotor.getKrakenX60(1);
            public static final DCMotor angleMotor = DCMotor.getFalcon500(1);
            public static final Voltage driveFrictionVoltage = Volts.of(0);
            public static final Voltage angleFrictionVoltage = Volts.of(0);
            public static final double wheelCoeffFriction = 1.2;
            public static final MomentOfInertia angleMomentOfInertia =
                KilogramSquareMeters.of(0.02);
            public static final Distance wheelRadius = Inches.of(1.9);
            public static final Current slipCurrent = Amps.of(120.0);

            public static final Current driveCurrentLimit = Amps.of(35.0);
            public static final Current driveCurrentLowerLimit = Amps.of(60.0);
            public static final Time driveCurrentLowerTimeThreshold = Seconds.of(0.1);
            public static final boolean driveEnableCurrentLimit = true;

            public static final Current angleCurrentLimit = Amps.of(25);
            public static final Current angleCurrentThreshold = Amps.of(40);
            public static final Time angleCurrentThresholdTime = Seconds.of(0.1);
            public static final boolean angleEnableCurrentLimit = true;

            public static final AngularVelocity maxSteerRate = RotationsPerSecond.of(4.0);
            public static final LinearAcceleration maxDriveRate = MetersPerSecondPerSecond.of(50.0);


            public static final double ffkS = 0.32;
            public static final double ffkV = 1.51;
            public static final double ffkT = 1.0 / driveMotor.KtNMPerAmp;
            public static final double ffkA = 0.27;
            public static final double drivekP = 0.12;
            public static final double drivekD = 0.0;
            public static final double anglekP = 100.0;
            public static final double anglekD = 0.0;
            public static final double driveReduction = Mk4iReductions.L1.reduction;
            public static final double angleReduction = Mk4iReductions.TURN.reduction;
        }

        public static final Mass robotMass = Pounds.of(120.0);

        /** Get config for Maple-Sim. */
        public static DriveTrainSimulationConfig getMapleConfig() {
            return DriveTrainSimulationConfig.Default().withRobotMass(robotMass)
                .withGyro(COTS.ofNav2X()).withCustomModuleTranslations(getModuleTranslations())
                .withSwerveModule(new SwerveModuleSimulationConfig(ModuleConstants.driveMotor,
                    ModuleConstants.angleMotor, ModuleConstants.driveReduction,
                    ModuleConstants.angleReduction, ModuleConstants.driveFrictionVoltage,
                    ModuleConstants.angleFrictionVoltage, ModuleConstants.wheelRadius,
                    ModuleConstants.angleMomentOfInertia, ModuleConstants.wheelCoeffFriction));
        }

        private enum Mk4iReductions {
            L1(8.14), L2(6.75), L3(6.12), TURN((150.0 / 7.0));

            final double reduction;

            Mk4iReductions(double reduction) {
                this.reduction = reduction;
            }
        }
    }

    /** Vision Constants */
    public static class Vision {

        /** Constants for an individual camera. */
        public static final record CameraConstants(String name, int height, int width,
            Rotation2d horizontalFieldOfView, Frequency framesPerSecond, Time latencyAvg,
            Time latencyStdDev, double calibErrorAvg, double calibErrorStdDev,
            Transform3d robotToCamera) {
        }

        public static final CameraConstants[] cameras = new CameraConstants[] {
            new CameraConstants("cam0", 1600, 1200, Rotation2d.fromDegrees(100), Hertz.of(20),
                Seconds.of(0.3), Seconds.of(0.02), 0.25, 0.08, new Transform3d())};

        public static final double zMargin = 0.75;
        public static final double fieldBorderMargin = 0.5;
    }
}
