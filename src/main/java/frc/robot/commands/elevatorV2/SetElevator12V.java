package frc.robot.commands.elevatorV2;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorV2.V2ElevatorSubsystem;

public class SetElevator12V extends Command {

    private final V2ElevatorSubsystem elevator;

    public SetElevator12V(V2ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        System.out.println("SetElevator12V called");
        elevator.setMotorVoltage(12);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
