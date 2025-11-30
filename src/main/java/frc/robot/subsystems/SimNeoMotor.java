package frc.robot.subsystems;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.BatterySim;

public class SimNeoMotor implements NeoMotor {

    private static final DCMotor motor = DCMotor.getNEO(1);
    private static final double J = 0.00032; // inertia in kg*m^2
    private double positionRotations = 0.0;

    private static final LinearSystem<N1, N1, N1> flywheelPlant = LinearSystemId.createFlywheelSystem(
            motor,                // motor
            J,
            1
        );

        private final FlywheelSim sim = new FlywheelSim(
            flywheelPlant,
            motor
    );


    private double voltage = 0.0;

    @Override
    public double getVelocity() {
        return sim.getAngularVelocityRPM();
    }

    @Override
    public void setVoltage(double volts) {
        this.voltage = volts;
    }

    @Override
    public double getAppliedOutput() {
        return voltage / 12.0;  // same range as Spark MAX (-1 to +1)
    }

    @Override
    public void updateSimulation(double dtSeconds) {
        sim.setInputVoltage(voltage);
        sim.update(dtSeconds);

        positionRotations += (sim.getAngularVelocityRadPerSec() * dtSeconds) / (2 * Math.PI);

        // simulate battery voltage sag
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));
    }

    @Override
    public double getBusVoltage() {
        // whatever the rio is currently simulated at rn
        return RoboRioSim.getVInVoltage();
    }

    @Override
    public double getPosition() {
        return positionRotations;
    }
}
