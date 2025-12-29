package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmConstants.ArmPositionStates;
import frc.robot.subsystems.arm.ArmConstants.GripperStates;
import frc.robot.subsystems.arm.ArmConstants.CargoStates;

public class EjectCone extends SequentialCommandGroup {

    public EjectCone(ArmSubsystem arm) {

        addRequirements(arm);

        addCommands(
            new ConditionalCommand(

                // Cargo is loaded
                new SequentialCommandGroup(

                    // stow > eject
                    new InstantCommand(() -> System.out.println("Starting cone eject sequence")),
                    new InstantCommand(() -> arm.setTargetPositionState(ArmPositionStates.EJECT)),
                    new InstantCommand(() -> System.out.println("Started stow > eject")),
                    new WaitUntilCommand(() -> arm.atPositionTarget()),

                    // open gripper
                    new InstantCommand(() -> arm.setGripperState(GripperStates.OPEN)),
                    new InstantCommand(() -> System.out.println("Opened gripper")),

                    // set cargo to loaded
                    new InstantCommand(() -> arm.setCargoState(CargoStates.EMPTY)),
                    new InstantCommand(() -> System.out.println("Updated cargo state to empty")),

                    // high > stow
                    new InstantCommand(() -> arm.setTargetPositionState(ArmPositionStates.STOW)),
                    new InstantCommand(() -> System.out.println("Started intake > stow")),
                    new WaitUntilCommand(() -> arm.atPositionTarget()),
                    new InstantCommand(() -> System.out.println("Cone intake finished"))
                ),

                // Cargo is empty
                new InstantCommand(() -> System.out.println("EjectCone skipped: no cargo loaded")),

                // Condition
                () -> (arm.getCargoState() == CargoStates.LOADED)

            )
        );
    }
}
