package frc.robot.subsystems;

public interface NeoMotor {

    double getVelocity();

    void setVoltage(double volts);

    default void updateSimulation(double dtSeconds) {};

    double getAppliedOutput();

    double getBusVoltage();

    double getPosition();
}
