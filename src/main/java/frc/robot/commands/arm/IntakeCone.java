package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmConstants.ArmPositionStates;
import frc.robot.subsystems.arm.ArmConstants.GripperStates;
import frc.robot.subsystems.arm.ArmConstants.CargoStates;

public class IntakeCone extends SequentialCommandGroup {

    public IntakeCone(ArmSubsystem arm) {

        addRequirements(arm);

        addCommands(
            new ConditionalCommand(

                // Cargo is empty
                new SequentialCommandGroup(

                    // stow > intake
                    new InstantCommand(() -> System.out.println("Started cone intake sequence")),
                    new InstantCommand(() -> arm.setGripperState(GripperStates.OPEN)),
                    new InstantCommand(() -> System.out.println("Opened gripper")),
                    new InstantCommand(() -> arm.setTargetPositionState(ArmPositionStates.INTAKE)),
                    new InstantCommand(() -> System.out.println("Started stow > intake")),
                    new WaitUntilCommand(() -> arm.atPositionTarget()),

                    // close gripper
                    new InstantCommand(() -> arm.setGripperState(GripperStates.CLOSED)),
                    new InstantCommand(() -> System.out.println("Closed gripper")),

                    // set cargo to loaded
                    new InstantCommand(() -> arm.setCargoState(CargoStates.LOADED)),
                    new InstantCommand(() -> System.out.println("Updated cargo state to loaded")),

                    // high > stow
                    new InstantCommand(() -> arm.setTargetPositionState(ArmPositionStates.STOW)),
                    new InstantCommand(() -> System.out.println("Started intake > stow")),
                    new WaitUntilCommand(() -> arm.atPositionTarget()),
                    new InstantCommand(() -> System.out.println("Cone intake finished"))
                ),

                // Cargo is loaded
                new InstantCommand(() -> System.out.println("IntakeCone skipped: cargo already loaded")),

                // Condition
                () -> arm.getCargoState() == CargoStates.EMPTY

            )
        );
    }
}
