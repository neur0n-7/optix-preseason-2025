package frc.robot.subsystems.arm.motor;

public interface ArmMotorIO {

    void setVoltage(double volts);

    double getVelocityRadPerSec();

    double getPositionDegrees();

    default void updateSimulation(double dtSeconds) { }

    /*
     * this is only needed for sim, to represent the arm getting 
     * slightly heavier after picking up the cone
    */
    default void setSimArmMass(boolean hasCone) { }

    
}