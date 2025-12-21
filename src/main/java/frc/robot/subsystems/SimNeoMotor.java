package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class SimNeoMotor implements NeoMotor {

    private static final DCMotor motor = DCMotor.getNEO(1);
    private static final double J = 0.00032; // inertia (kg*m^2)
    
    private static final LinearSystem<N1, N1, N1> plant =
        LinearSystemId.createFlywheelSystem(motor, J, 1.0);

    private final FlywheelSim sim = new FlywheelSim(plant, motor);

    private double positionRotations = 0.0;
    private double voltage = 0.0;

    @Override
    public double getVelocity() {
        return sim.getAngularVelocityRPM();
    }

    @Override
    public void setVoltage(double volts) {
        voltage = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public double getAppliedOutput() {
        return voltage / 12.0;
    }

    @Override
    public void updateSimulation(double dtSeconds) {

        // System.out.println("updateSimulation() called");

        sim.setInputVoltage(voltage);
        sim.update(dtSeconds);

        double radPerSec = sim.getAngularVelocityRadPerSec();
        SmartDashboard.putNumber("ElevatorV2/RadsPerSec", radPerSec);
        SmartDashboard.putNumber("ElevatorV2/DtSeconds", dtSeconds);
        

        positionRotations += (radPerSec * dtSeconds) / (2.0 * Math.PI);

        // battery sag
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps())
        );
    }

    @Override
    public double getBusVoltage() {
        return RoboRioSim.getVInVoltage();
    }

    @Override
    public double getPosition() {
        SmartDashboard.putNumber("ElevatorV2/MotorRotations", positionRotations);
        return positionRotations;
    }
}
