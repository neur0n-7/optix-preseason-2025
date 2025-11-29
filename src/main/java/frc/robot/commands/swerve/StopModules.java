
package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class StopModules extends Command {

    private final SwerveSubsystem swerve;

    public StopModules(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.drive(0, 0, 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0, false);
    }
}
