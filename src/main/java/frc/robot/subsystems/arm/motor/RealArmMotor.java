package frc.robot.subsystems.arm.motor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.arm.ArmConstants;

import com.revrobotics.spark.SparkMax;

public class RealArmMotor implements ArmMotorIO {

    private final SparkMax motor;
    private final RelativeEncoder encoder;

    public RealArmMotor(int canId) {
        motor = new SparkMax(canId, MotorType.kBrushless);
        encoder = motor.getEncoder();
        encoder.setPosition(0.0);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public double getVelocityRadPerSec() {
        // Returns degrees per second
        double motorRpm = encoder.getVelocity();
        double armDegreesPerMin = motorRpm * 360.0 / ArmConstants.gearing;
        return Units.degreesToRadians(armDegreesPerMin / 60) ;
    }

    @Override
    public double getPositionDegrees() {
        // Returns degrees
        return encoder.getPosition() * 360.0 / ArmConstants.gearing;
    }
}
