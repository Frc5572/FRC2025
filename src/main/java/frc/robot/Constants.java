package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
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
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
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

    public static final boolean shouldDrawStuff = false;

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
     * Current threshold that indicates an algae is in possestion
     */
    public static final double HAS_ALGAE_CURRENT_THRESHOLD = 100;

    /**
     * Algae misc values
     */
    public static final class Algae {
        public static final int VOLTAGE = 2;
        public static final int NEGATIVE_VOLTAGE = -2;
    }

    // pit and alt operator controllers
    public static final int PIT_CONTROLLER_ID = 3;
    public static final int ALT_OPERATOR_ID = 2;



    /**
     * Motor CAN id's.
     */
    public static final class Motors {
        /**
         * Primary Coral Scoring CAN id's
         */
        public static final class PrimaryCoralScoring {
            public static final int Coral_Scoring_NEO_ID = 5;
        }

        /**
         * Algae Motor CAN id's
         */
        public static final class AlgaeMotors {
            public static final int ALGAE_MOTOR_ID = 6;
        }

    }


    /**
     * leds constants class
     */
    public static final class LEDs {
        public static final int LED_PORT = 0;
        public static final int LED_LENGTH = 123;
    }


    /**
     *
     * Climb Constants.
     */
    public static final class Climb {

        public static final int LEFT_TALON_FX_ID = 3;
        public static final int RIGHT_TALON_FX_ID = 12;
        public static final int TOUCH_SENSOR_CHANNEL = 2;
        public static final Angle CLIMB_ANGLE = Radians.of(550);
        public static final Angle MAX_ANGLE = Radians.of(703);
        public static final Angle CLIMBER_OUT_ANGLE = Radians.of(260);
        public static final Angle CLIMBER_START_ANGLE = Radians.of(140);
        public static final double GEAR_RATIO = 1;
        public static final double CLIMB_VOLTAGE = 3.0;
        public static final double PRE_CLIMB_VOLTAGE = 5.5;
        public static final double RESET_VOLTAGE = -5.5;

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

        public static final Translation2d[] moduleTranslations = new Translation2d[] {
            new Translation2d(wheelBase.in(Meters) / 2, trackWidth.in(Meters) / 2),
            new Translation2d(wheelBase.in(Meters) / 2, -trackWidth.in(Meters) / 2),
            new Translation2d(-wheelBase.in(Meters) / 2, trackWidth.in(Meters) / 2),
            new Translation2d(-wheelBase.in(Meters) / 2, -trackWidth.in(Meters) / 2)};

        /**
         * Swerve Kinematics No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(moduleTranslations);

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        public static final InvertedValue driveMotorInvert =
            InvertedValue.CounterClockwise_Positive;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert =
            SensorDirectionValue.CounterClockwise_Positive;

        /* Swerve Current Limiting */


        /**
         * These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.0;
        public static final double AUTO_MAX_SPEED = 3.0;
        public static final double MAX_ELEVATOR_SPEED = 2.0;

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
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 1; // duplicate?
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.008789);

        }

        /**
         * Front Right Module - Module 1
         */
        public static final class Mod1 {
            public static final int driveMotorID = 9;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 2; // duplicate?
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.298096);

        }

        /**
         * Back Left Module - Module 2
         */
        public static final class Mod2 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 19;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.451172);

        }

        /**
         * Back Right Module - Module 3
         */
        public static final class Mod3 {
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 10;
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
        public static final Distance bumperFront = Inches.of(17.5);
        public static final Distance bumperRight = Inches.of(17.5);

        /** Get config for Maple-Sim. */
        public static DriveTrainSimulationConfig getMapleConfig() {
            return DriveTrainSimulationConfig.Default().withRobotMass(robotMass)
                .withGyro(COTS.ofNav2X()).withCustomModuleTranslations(moduleTranslations)
                .withBumperSize(bumperFront.times(2), bumperRight.times(2))
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


    /**
     * Elevator Constants
     */

    public static final class Elevator {
        public static final int RIGHT_ID = 7;
        public static final int LEFT_ID = 4;
        public static final int LIMIT_ID = 1;

        public static final NeutralModeValue BREAK = NeutralModeValue.Brake;

        // PID and feedforward
        public static final double KP = 7.0;
        public static final double KI = 2.0;
        public static final double KD = 0.0;
        public static final double KS = 0.1675;
        public static final double KV = 0.0;
        public static final double KA = 0.0;
        public static final double KG = 0.3375;
        public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(0.0);

        // positions
        public static final Distance HOME = Inches.of(2);
        public static final Distance P0 = Inches.of(16.5); // Algae L2-L3
        public static final Distance P1 = Inches.of(26); // Coral L2
        public static final Distance P2 = Inches.of(33); // Algae L3-L4
        public static final Distance P3 = Inches.of(43); // Coral L3
        public static final Distance P4 = Inches.of(67); // Coral L4


        public static final double gearRatio = 1.0;
        public static final Distance INCHES_AT_TOP = Inches.of(72.0);
        public static final Angle ROTATIONS_AT_TOP = Radians.of(220);
        public static final double SensorToMechanismRatio =
            Constants.Elevator.ROTATIONS_AT_TOP.in(Rotations)
                / Constants.Elevator.INCHES_AT_TOP.in(Meters);

        public static final String heightName = "elevator height";
    }


    /** Vision Constants */
    public static class Vision {

        public static final AprilTagFieldLayout fieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        /** Constants for an individual camera. */
        public static final record CameraConstants(String name, int height, int width,
            Rotation2d horizontalFieldOfView, Frequency framesPerSecond, Time latencyAvg,
            Time latencyStdDev, double calibErrorAvg, double calibErrorStdDev,
            Transform3d robotToCamera) {
            public double centerPixelYawRads() {
                return robotToCamera.getRotation().getZ();
            }
        }

        public static final CameraConstants[] cameras = new CameraConstants[] {
            new CameraConstants("cam0", 800, 1280, Rotation2d.fromDegrees(100), Hertz.of(20),
                Seconds.of(0.3), Seconds.of(0.02), 0.25, 0.08,
                new Transform3d(
                    new Translation3d(Units.inchesToMeters(11), -Units.inchesToMeters(12), 0),
                    new Rotation3d(Math.PI, 0, 0)))};

        public static final double zMargin = 0.75;
        public static final double fieldBorderMargin = 0.5;
    }


    /** State Estimator Constants */
    public static class StateEstimator {
        public static final boolean keepInField = true;
        public static final boolean keepOutOfReefs = true;
        public static final double visionTrust = 0.02;
        public static final double visionTrustRotation = 0.02;
    }

    /**
     * Primary Coral Scoring Constants
     */
    public static final class CoralScoringConstants {
        public static final int Scoring_Beam_Brake_DIO_Port = 0;
        public static final int Intake_Beam_Breake_DIO_Port = 3;
        public static final int Random_Touch_Sensor = 1;
    }
}


