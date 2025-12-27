package frc.robot.subsystems.elevator;

public interface ElevatorMotorIO {

    void setVoltage(double volts);

    double getVelocity();

    double getPositionMeters();

    default void updateSimulation(double dtSeconds){};
}