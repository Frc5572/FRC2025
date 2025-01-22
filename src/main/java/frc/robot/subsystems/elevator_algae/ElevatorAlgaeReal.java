package frc.robot.subsystems.elevator_algae;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

/*
 * Algae Real Class
 */
public class ElevatorAlgaeReal implements ElevatorAlgaeIO {
    private final SparkFlex AlgaeMotor = // Algae motor
        new SparkFlex(Constants.Motors.AlgaeMotors.INTAKE_MOTOR_ID, MotorType.kBrushless);


    @Override
    public void updateInputs(AlgaeIOInputs inputs) { // update inputs to IO layer
        inputs.AlgaeMotorSpeed = AlgaeMotor.get();
    }

    @Override
    public void setAlgaeMotorSpeed(double speed) { // set hardware speed
        AlgaeMotor.set(speed);
    }

}
