package frc.robot.subsystems;

// Interface to use either real vs simulated shooter motor
public interface NeoMotor {

    double getVelocity();

    void setVoltage(double volts);

    default void updateSimulation(double dtSeconds) {};

    double getAppliedOutput();

    double getBusVoltage();

    double getPosition();
}
