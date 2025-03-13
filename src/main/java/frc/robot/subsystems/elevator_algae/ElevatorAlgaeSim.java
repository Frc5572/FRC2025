package frc.robot.subsystems.elevator_algae;

import static edu.wpi.first.units.Units.RPM;
import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Simulated Algae Subsystem
 */
public class ElevatorAlgaeSim implements ElevatorAlgaeIO {
    private DCMotorSim motor =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(.01, .01), DCMotor.getNeoVortex(1));

    /**
     * Simulated Algae Subsystem
     */
    public ElevatorAlgaeSim() {}

    public void updateInputs(AlgaeIOInputs inputs) {
        motor.update(LoggedRobot.defaultPeriodSecs);
        inputs.algaeMotorCurrent = motor.getCurrentDrawAmps();
        inputs.motorRPM = RPM.of(motor.getAngularVelocityRPM());
    }

    public void setAlgaeMotorVoltage(double volts) {
        motor.setInputVoltage(volts);
    }
}
