package frc.robot.subsystems;

public interface NeoMotor {

    double getVelocity();

    void setVoltage(double volts);

    // by default, updateSimulation() won't do anything in real
    default void updateSimulation(double dtSeconds) { };

    double getAppliedOutput();

    double getBusVoltage();

    double getPosition();
}
