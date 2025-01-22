package frc.robot.subsystems.PrimaryCoralScoring;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class CoralScoring extends SubsystemBase {
    private CoralScoringIO io;
    private CoralScoringInputsAutoLogged CoralScoringAutoLogged =
        new CoralScoringInputsAutoLogged();

    public CoralScoring(CoralScoringIO io) {
        this.io = io;
        io.updateInputs(CoralScoringAutoLogged);
    }

    @Override
    public void periodic() {
        io.updateInputs(CoralScoringAutoLogged);
        Logger.processInputs("Coral Scoring", CoralScoringAutoLogged);
    }

    public void setScoringMotor(double percentage) {
        Logger.recordOutput("Scoring Percentage", percentage);
        io.setCoralScoringMotorPercentage(percentage);
    }

    public boolean getScoringBeamBrakeStatus() {
        return CoralScoringAutoLogged.scoringBeamBrake;
    }

    public Command runPreScoringMotor(double ScoringSpeed) {
        return Commands.startEnd(() -> {
            setScoringMotor(ScoringSpeed);
        }, () -> {
            setScoringMotor(0);
        }, this).until(() -> getScoringBeamBrakeStatus());
    }

    public Command runScoringMotor(double ScoringSpeed) {
        return Commands.startEnd(() -> {
            setScoringMotor(ScoringSpeed);
        }, () -> {
            setScoringMotor(0);
        }, this).until(() -> !getScoringBeamBrakeStatus());
    }


}
