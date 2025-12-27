package frc.robot.subsystems.arm.gripper;

import frc.robot.subsystems.arm.ArmConstants.CargoStates;
import frc.robot.subsystems.arm.ArmConstants.GripperStates;

public interface GripperIO {

    void setGripperState(GripperStates targetGripperState);

    GripperStates getGripperState();

    // set if holding cone or not
    void setCargoState(CargoStates targetCargoState);

    CargoStates getCargoState();
}