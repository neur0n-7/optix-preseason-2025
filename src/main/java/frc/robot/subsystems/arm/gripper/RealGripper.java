package frc.robot.subsystems.arm.gripper;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.arm.ArmConstants.CargoStates;
import frc.robot.subsystems.arm.ArmConstants.GripperStates;

public class RealGripper implements GripperIO {

    private final Solenoid gripperSolenoid;
    private GripperStates gripperState = GripperStates.OPEN;
    private CargoStates cargoState = CargoStates.EMPTY;

    public RealGripper(PneumaticsModuleType moduleType, int channel) {
        gripperSolenoid = new Solenoid(moduleType, channel);
    }

    @Override
    public void setGripperState(GripperStates targetGripperState){
        gripperState = targetGripperState;
        gripperSolenoid.set(gripperState.isClosed); // closed = true, on
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
