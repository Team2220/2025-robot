package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class Elevator {
    TalonFX elevatorMotorR;
    TalonFX elevatorMotorL;
    TalonFXConfiguration configR;
    TalonFXConfiguration configL;
    double gearRatio;

    public Elevator() {
        elevatorMotorR = new TalonFX(1); // on the right of the elevator (inverted)
        elevatorMotorL = new TalonFX(2); // on the left of the elevator
        gearRatio = 0; // change when we know the gear ratio

        configR = new TalonFXConfiguration();
        configL = new TalonFXConfiguration();

        configR.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configR.Feedback.SensorToMechanismRatio = gearRatio;
        configL.Feedback.SensorToMechanismRatio = gearRatio;

        elevatorMotorR.getConfigurator().apply(configR);
        elevatorMotorL.getConfigurator().apply(configL);
    }
}
