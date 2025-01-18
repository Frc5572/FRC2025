package frc.robot.subsystems.ElevatorAlgae;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

// Algae Real Class
public class ElevatorAlgaeReal implements ElevatorAlgaeIO {
    private final SparkFlex AlgaeMotor =
        new SparkFlex(Constants.Motors.AlgaeMotors.INTAKE_MOTOR_ID, MotorType.kBrushless);


    @Override
    public void updateInputs(AlgaeIOInputs inputs) {
        inputs.AlgaeMotorSpeed = AlgaeMotor.get();
    }

    @Override
    public void setAlgaeMotorSpeed(double speed) {
        AlgaeMotor.set(speed);
    }

}
