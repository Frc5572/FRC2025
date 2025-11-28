package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class AntiTilt {
    private final static Angle maxTiltAngle = Degrees.of(15.0);
    private final static Angle criticalTiltAngle = Degrees.of(20.0);
    private final static double kP = 0.8;
    private final static LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(3.0);
    private final static LinearAcceleration minDeceleration = MetersPerSecondPerSecond.of(4.0);

    private static ChassisSpeeds previousSpeeds = new ChassisSpeeds();
    private static double previousTime = 0;


    public static ChassisSpeeds getSafeSpeeds(ChassisSpeeds desirSpeeds, Distance elevatorHeight,
        Angle pitch, Angle roll) {
        double curretTime = Timer.getFPGATimestamp();
        double deltaTime = curretTime - previousTime;

        double heightFactor =
            1.0 - (elevatorHeight.in(Meters) / Constants.Elevator.BARGE_HEIGHT.in(Meters)) * 0.4;
        LinearAcceleration adjustedMaxAccel = maxAcceleration.times(heightFactor);
        LinearAcceleration adjustedMinDecel = minDeceleration.times(heightFactor);

        ChassisSpeeds limitSpeeds = limitAcceleration(desirSpeeds, previousSpeeds, deltaTime,
            adjustedMaxAccel.in(MetersPerSecondPerSecond),
            adjustedMinDecel.in(MetersPerSecondPerSecond));
        ChassisSpeeds correctedSpeed =
            applyTiltCorrection(limitSpeeds, pitch.in(Degrees), roll.in(Degrees));
        previousSpeeds = correctedSpeed;
        previousTime = curretTime;

        return correctedSpeed;
    }

    public static ChassisSpeeds limitAcceleration(ChassisSpeeds desiredSpeeds,
        ChassisSpeeds previousSpeeds, double deltaTime, double maxAccel, double maxDecel) {
        if (deltaTime <= 0)
            return desiredSpeeds;
        double vxAccel = (desiredSpeeds.vxMetersPerSecond - previousSpeeds.vxMetersPerSecond);
        double vyAccel = (desiredSpeeds.vyMetersPerSecond - previousSpeeds.vyMetersPerSecond);
        double maxAllowedVx = previousSpeeds.vxMetersPerSecond;

        if (vxAccel > 0) {
            maxAllowedVx += maxAccel * deltaTime;
        } else {
            maxAllowedVx -= maxDecel * deltaTime;
        }
        double limitedVx = clamp(desiredSpeeds.vxMetersPerSecond,
            Math.min(previousSpeeds.vxMetersPerSecond, maxAllowedVx),
            Math.max(previousSpeeds.vxMetersPerSecond, maxAllowedVx));

        double maxAllowedVy = previousSpeeds.vyMetersPerSecond;
        if (vyAccel > 0) {
            maxAllowedVy += maxAccel * deltaTime;
        } else {
            maxAllowedVy -= maxDecel * deltaTime;
        }
        double limitedVy = clamp(desiredSpeeds.vyMetersPerSecond,
            Math.min(previousSpeeds.vyMetersPerSecond, maxAllowedVy),
            Math.max(previousSpeeds.vyMetersPerSecond, maxAllowedVy));
        return new ChassisSpeeds(limitedVx, limitedVy, desiredSpeeds.omegaRadiansPerSecond);
    }

    private static ChassisSpeeds applyTiltCorrection(ChassisSpeeds speeds, double pitchDeg,
        double rollDeg) {
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        double pitchCorrection = calculateCorrection(pitchDeg);
        double rollCorrection = calculateCorrection(rollDeg);
        vx -= pitchDeg * pitchCorrection;
        vy -= rollDeg * rollCorrection;
        if (Math.abs(pitchDeg) > criticalTiltAngle.in(Degrees)) {
            vx = -Math.signum(pitchDeg) * 2.0;
        }
        if (Math.abs(rollDeg) > criticalTiltAngle.in(Degrees)) {
            vy = -Math.signum(rollDeg) * 2.0;
        }

        return new ChassisSpeeds(vx, vy, speeds.omegaRadiansPerSecond);
    }

    private static double calculateCorrection(double tiltDeg) {
        double absTilt = Math.abs(tiltDeg);

        if (absTilt < maxTiltAngle.in(Degrees)) {
            return 0.0;
        } else if (absTilt < criticalTiltAngle.in(Degrees)) {
            return kP * (absTilt - maxTiltAngle.in(Degrees))
                / (criticalTiltAngle.in(Degrees) - maxTiltAngle.in(Degrees));
        } else {
            return kP;
        }
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
