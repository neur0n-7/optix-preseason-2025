package frc.robot.subsystems.arm.gripper;

import frc.robot.subsystems.arm.ArmConstants.CargoStates;
import frc.robot.subsystems.arm.ArmConstants.GripperStates;

public class SimGripper implements GripperIO {

    private GripperStates gripperState = GripperStates.OPEN;
    private CargoStates cargoState = CargoStates.EMPTY;

    @Override
    public void setGripperState(GripperStates targetGripperState) {
        gripperState = targetGripperState;
    }

    @Override
    public GripperStates getGripperState() {
        return gripperState;
    }

    @Override
    public void setCargoState(CargoStates targetCargoState) {
        cargoState = targetCargoState;
    }

    @Override
    public CargoStates getCargoState() {
        return cargoState;
    }
}
