package frc.robot.commands.elevatorV2;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorV2.V2ElevatorSubsystem;
import frc.robot.subsystems.elevatorV2.V2ElevatorConstants.ElevatorStates;

public class SetElevatorState extends Command {

    private final V2ElevatorSubsystem elevator;
    private final ElevatorStates target;

    public SetElevatorState(V2ElevatorSubsystem elevator, ElevatorStates state) {
        this.elevator = elevator;
        this.target = state;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setTargetState(target);
    }

    @Override
    public boolean isFinished() {
        return elevator.atTarget();
    }
}
