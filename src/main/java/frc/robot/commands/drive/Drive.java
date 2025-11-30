
package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Drive extends Command {

  public Drive(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  private final DriveSubsystem m_subsystem;

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setMotorVoltagePID(5.0);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}