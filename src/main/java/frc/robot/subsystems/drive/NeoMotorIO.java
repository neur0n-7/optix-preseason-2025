package frc.robot.subsystems.drive;

public interface NeoMotorIO {

    double getVelocity();

    void setVoltage(double volts);

    // by default, updateSimulation() won't do anything in real
    default void updateSimulation(double dtSeconds) { };

    double getAppliedOutput();

    double getBusVoltage();

    double getPositionMeters();
}
