package frc.robot.subsystems.arm.motor;

public interface ArmMotorIO {

    void setVoltage(double volts);

    double getVelocity();

    double getPositionDegrees();

    default void updateSimulation(double dtSeconds) { };    

}