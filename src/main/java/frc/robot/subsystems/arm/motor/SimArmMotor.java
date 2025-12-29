package frc.robot.subsystems.arm.motor;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.arm.ArmConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;

public class SimArmMotor implements ArmMotorIO {

    private SingleJointedArmSim sim;
    private double voltage = 0.0;

    public SimArmMotor() {

        double momentOfInertia = ArmConstants.totalLoadedArmMassKg * Math.pow(ArmConstants.shoulderLength, 2);

        sim = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                ArmConstants.gearing,
                momentOfInertia,
                ArmConstants.shoulderLength,
                Units.degreesToRadians(0),
                Units.degreesToRadians(20000),
                true,
                Units.degreesToRadians(ArmConstants.ArmPositionStates.STOW.position_degs)
        );
    }

    @Override
    public void setVoltage(double volts) {
        voltage = MathUtil.clamp(volts, -12.0, 12.0);
        sim.setInputVoltage(voltage);
    }

    @Override
    public double getVelocityRadPerSec() {
        return sim.getVelocityRadPerSec();
    }

    @Override
    public double getPositionDegrees() {
        return Math.toDegrees(sim.getAngleRads());
    }

    public void updateSimulation(double dtSeconds) {
        sim.update(dtSeconds);
    }

    public void setSimArmMass(boolean hasCone) {
        double armMass = hasCone ? ArmConstants.totalLoadedArmMassKg : ArmConstants.totalEmptyArmMassKg;
        double momentOfInertia = armMass * Math.pow(ArmConstants.shoulderLength, 2);
    
        double currentAngle = sim.getAngleRads();
        double currentVelocity = sim.getVelocityRadPerSec();
    
        // recreate sim with new mass
        sim = new SingleJointedArmSim(
            DCMotor.getNEO(1),
            ArmConstants.gearing,
            momentOfInertia,
            ArmConstants.shoulderLength,
            Units.degreesToRadians(0),
            Units.degreesToRadians(20000),
            true,
            currentAngle
        );
    
        sim.setState(currentAngle, currentVelocity);
    }
    
}
