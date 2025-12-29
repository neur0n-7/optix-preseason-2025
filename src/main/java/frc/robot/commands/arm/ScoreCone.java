package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmConstants.ArmPositionStates;
import frc.robot.subsystems.arm.ArmConstants.CargoStates;
import frc.robot.subsystems.arm.ArmConstants.GripperStates;

public class ScoreCone extends SequentialCommandGroup {

    public ScoreCone(ArmSubsystem arm, ArmPositionStates scoreTargetState){
        
        addCommands(
            // stow > high
            new InstantCommand(() -> System.out.println("Started score cone sequence (%s)".formatted(scoreTargetState.toString()))),
            new InstantCommand(() -> arm.setTargetPositionState(scoreTargetState)),
            new InstantCommand(() -> System.out.println("Started stow > target state (%s)".formatted(scoreTargetState.toString()))),
            new WaitUntilCommand(() -> arm.atPositionTarget()),

            // open gripper
            new InstantCommand(() -> arm.setGripperState(GripperStates.OPEN)),
            new InstantCommand(() -> System.out.println("Opened gripper")),

            // set cargo to empty
            new InstantCommand(() -> arm.setCargoState(CargoStates.EMPTY)),
            new InstantCommand(() -> System.out.println("Updated cargo state to empty")),

            // high > stow
            new InstantCommand(() -> arm.setTargetPositionState(ArmPositionStates.STOW)),
            new InstantCommand(() -> System.out.println("Started target state (%s) > stow".formatted(scoreTargetState.toString()))),
            new WaitUntilCommand(() -> arm.atPositionTarget()),
            new InstantCommand(() -> System.out.println("Score cone finished"))
        );
    }
}
