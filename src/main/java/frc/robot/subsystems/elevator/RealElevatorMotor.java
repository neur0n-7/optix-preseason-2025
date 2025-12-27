package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class RealElevatorMotor implements ElevatorMotorIO {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final boolean isReversed;

    public RealElevatorMotor(int canId, boolean isReversed) {
        motor = new SparkMax(canId, MotorType.kBrushless);
        encoder = motor.getEncoder();
        encoder.setPosition(0.0);

        this.isReversed = isReversed;
    }

    public RealElevatorMotor(int canId) {
        motor = new SparkMax(canId, MotorType.kBrushless);
        encoder = motor.getEncoder();
        encoder.setPosition(0.0);

        this.isReversed = false;
    }


    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(isReversed ? -1: 1 * volts);
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity() * ElevatorConstants.metersPerMotorRotation;
    }

    @Override
    public double getPositionMeters() {
        return encoder.getPosition() * ElevatorConstants.metersPerMotorRotation;
    }
}
