
package frc.robot.commands.driveV2;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveV2.DriveSubsystemV2;

public class GoToDegrees extends Command {

  private final DriveSubsystemV2 m_subsystem;
  private final double degrees;

  public GoToDegrees(DriveSubsystemV2 subsystem, double degrees) {
    this.m_subsystem = subsystem;
    this.degrees = degrees;
    addRequirements(subsystem);
  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setDegreeSetpoint(degrees);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}