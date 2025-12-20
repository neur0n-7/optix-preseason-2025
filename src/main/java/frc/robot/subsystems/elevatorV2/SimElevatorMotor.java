package frc.robot.subsystems.elevatorV2;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.NeoMotor;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.math.MathUtil;

public class SimElevatorMotor implements NeoMotor {

    private final ElevatorSim sim;
    private double voltage = 0.0;

    public SimElevatorMotor() {
        sim = new ElevatorSim(
            DCMotor.getNEO(1),
            V2ElevatorConstants.gearing,
            V2ElevatorConstants.carriageMassKg,
            V2ElevatorConstants.drumDiameter / 2.0,
            V2ElevatorConstants.elevatorBaseHeight,
            V2ElevatorConstants.elevatorBaseHeight + V2ElevatorConstants.ElevatorStates.HIGHEST.position,
            true,
            V2ElevatorConstants.elevatorBaseHeight
        );
    }

    @Override
    public void setVoltage(double volts) {
        voltage = MathUtil.clamp(volts, -12.0, 12.0);
        sim.setInputVoltage(voltage);
    }

    @Override
    public double getBusVoltage() {
        return RoboRioSim.getVInVoltage();
    }

    @Override
    public double getAppliedOutput() {
        return voltage / 12.0;
    }

    @Override
    public double getVelocity() {
        return sim.getVelocityMetersPerSecond();
    }

    @Override
    public double getPosition() {
        return sim.getPositionMeters() / V2ElevatorConstants.metersPerMotorRotation;
    }

    @Override
    public void updateSimulation(double dtSeconds) {
        sim.update(dtSeconds);
    }
}
