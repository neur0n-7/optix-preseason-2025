
package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class StopDriving extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  public StopDriving(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  
  private final DriveSubsystem m_subsystem;

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setMotorVoltage(0);
    System.out.println("Motor has been stopped.");
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}