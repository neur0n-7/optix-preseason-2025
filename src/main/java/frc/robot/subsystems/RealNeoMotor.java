package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class RealNeoMotor implements NeoMotor {

    private final SparkMax motor;

    public RealNeoMotor(int canId) {
        motor = new SparkMax(canId, MotorType.kBrushless);
    }

    @Override
    public double getVelocity() {
        return motor.getEncoder().getVelocity(); // RPM
    }

    @Override
    public double getAppliedOutput() {
        return motor.getAppliedOutput();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts); // volts to motor percent output
    }

    @Override
    public double getBusVoltage() {
        return motor.getBusVoltage();
    }

    @Override
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }
}
