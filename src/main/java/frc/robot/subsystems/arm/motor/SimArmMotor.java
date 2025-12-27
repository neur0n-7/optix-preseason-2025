package frc.robot.subsystems.arm.motor;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.arm.ArmConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;

public class SimArmMotor implements ArmMotorIO {

    private final SingleJointedArmSim sim;
    private double voltage = 0.0;

    public SimArmMotor() {

        double momentOfInertia = ArmConstants.totalEmptyArmMassKg * Math.pow(ArmConstants.shoulderLength, 2);

        sim = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                ArmConstants.gearing,
                momentOfInertia,
                ArmConstants.shoulderLength,
                Units.degreesToRadians(0),
                Units.degreesToRadians(180),
                true,
                Units.degreesToRadians(ArmConstants.ArmPositionStates.STOW.position_degs)
        );
    }

    @Override
    public void setVoltage(double volts) {
        voltage = MathUtil.clamp(volts, -12.0, 12.0);
        sim.setInput(voltage);
    }

    @Override
    public double getVelocity() {
        return sim.getVelocityRadPerSec();
    }

    @Override
    public double getPositionDegrees() {
        return Math.toDegrees(sim.getAngleRads());
    }

    public void updateSimulation(double dtSeconds) {
        sim.update(dtSeconds);
    }
}
