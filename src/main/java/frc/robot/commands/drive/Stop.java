// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class Stop extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  public Stop(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  
  private final DriveSubsystem m_subsystem;

  /**
   * Creates a new Drive.
   *
   * @param subsystem The subsystem used by this command.
   */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setMotorVoltage(0);
    System.out.println("Motor has been stopped.");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}