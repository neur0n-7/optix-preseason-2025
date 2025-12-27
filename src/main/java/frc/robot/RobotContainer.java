// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.swerve.JoystickDrive;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.commands.drive.GoToDegrees;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.RealNeoMotor;
import frc.robot.subsystems.drive.SimNeoMotor;
import frc.robot.subsystems.elevator.SimElevatorMotor;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.RealElevatorMotor;

public class RobotContainer {

	Map<String, Boolean> subsystemEnabled = new HashMap<>();

	// DRIVE
	private final DriveSubsystem m_DriveSubsystem;
	private final GoToDegrees m_GoTo90Degrees;
	private final GoToDegrees m_GoTo0Degrees;

	// SWERVE
	private final SwerveSubsystem m_SwerveSubsystem;
	private final JoystickDrive m_JoystickDrive;
	private final DoubleSupplier xSpeedSupplier;
	private final DoubleSupplier ySpeedSupplier;
	private final DoubleSupplier rotSpeedSupplier;

	// ELEVATOR
	private final ElevatorSubsystem m_ElevatorSubsystem;
	private final SetElevatorState m_SetElevatorHighest;
	private final SetElevatorState m_SetElevatorLowest;
	private final SetElevatorState m_SetElevatorMiddle;

	private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

	private final CommandXboxController m_driverController = new CommandXboxController(
			OperatorConstants.kDriverControllerPort);

	public RobotContainer() {

		// ENABLE SUBSYTEMS
		// //////////////////////////////////////////////////////////////////////
		subsystemEnabled.put("SWERVE", false);
		subsystemEnabled.put("ELEVATOR", true);
		subsystemEnabled.put("DRIVE", true);
		/////////////////////////////////////////////////////////////////////////////////////////

		// DRIVE
		if (subsystemEnabled.getOrDefault("DRIVE", false)) {
			if (RobotBase.isSimulation()) {
				m_DriveSubsystem = new DriveSubsystem(new SimNeoMotor());
			} else {
				m_DriveSubsystem = new DriveSubsystem(new RealNeoMotor(OperatorConstants.driveMotorCanId));
			}

			m_GoTo90Degrees = new GoToDegrees(m_DriveSubsystem, 90);
			m_GoTo0Degrees = new GoToDegrees(m_DriveSubsystem, 0);
		} else {
			m_DriveSubsystem = null;
			m_GoTo90Degrees = null;
			m_GoTo0Degrees = null;
		}

		// ELEVATOR
		if (subsystemEnabled.getOrDefault("ELEVATOR", false)) {
			if (RobotBase.isSimulation()){
				m_ElevatorSubsystem = new ElevatorSubsystem(new SimElevatorMotor());
			} else {
				m_ElevatorSubsystem = new ElevatorSubsystem(new RealElevatorMotor(Constants.OperatorConstants.elevatorMotorCanId, false));
			}
	
			m_SetElevatorLowest = new SetElevatorState(m_ElevatorSubsystem, ElevatorConstants.ElevatorStates.LOWEST);
			m_SetElevatorMiddle = new SetElevatorState(m_ElevatorSubsystem, ElevatorConstants.ElevatorStates.MIDDLE);
			m_SetElevatorHighest = new SetElevatorState(m_ElevatorSubsystem, ElevatorConstants.ElevatorStates.HIGHEST);
		} else {
			m_ElevatorSubsystem = null;
			m_SetElevatorLowest = null;
			m_SetElevatorMiddle = null;
			m_SetElevatorHighest = null;
		}

		// SWERVE
		if (subsystemEnabled.getOrDefault("SWERVE", false)) {
			m_SwerveSubsystem = new SwerveSubsystem(RobotBase.isReal());

			xSpeedSupplier = () -> -MathUtil.applyDeadband(
					m_driverController.getLeftY(), 0.1)
					* SwerveConstants.ControlConstants.teleopMaxSpeedMetersPerSecond;

			ySpeedSupplier = () -> MathUtil.applyDeadband(
					m_driverController.getLeftX(), 0.1)
					* SwerveConstants.ControlConstants.teleopMaxSpeedMetersPerSecond;

			rotSpeedSupplier = () -> MathUtil.applyDeadband(
					m_driverController.getRightX(), 0.1)
					* SwerveConstants.ControlConstants.teleopMaxAngularSpeedRadPerSecond;

			m_JoystickDrive = new JoystickDrive(m_SwerveSubsystem, xSpeedSupplier, ySpeedSupplier, rotSpeedSupplier);
		} else {
			m_SwerveSubsystem = null;
			m_JoystickDrive = null;
			xSpeedSupplier = null;
			ySpeedSupplier = null;
			rotSpeedSupplier = null;
		}

		configureBindings();
	}

	private void configureBindings() {
		/*
		 * The bindings are as follows:
		 * 
		 * SWERVE
		 * - Left joystick to move robot w/ swerve
		 * - Right joystick to rotate robot w/ swerve
		 * 
		 * ELEVATOR
		 * - X to go to elevator max
		 * - Y to go to elevator min
		 * - A to go to elevator mid
		 * 
		 * DRIVE (single motor):
		 * - Press X to go to 0 degrees
		 * - Press Y to go to 90 degrees
		 */

		if (subsystemEnabled.getOrDefault("SWERVE", false)) {
			// SWERVE
			System.out.println("SWERVE - Subsystem enabled");
			m_SwerveSubsystem.setDefaultCommand(m_JoystickDrive);
		}
		;

		if (subsystemEnabled.getOrDefault("ELEVATOR", false)) {
			// ELEVATOR
			System.out.println("ELEVATOR - Subsystem enabled");
			m_driverController.x().onTrue(m_SetElevatorHighest);
			m_driverController.y().onTrue(m_SetElevatorLowest);
			m_driverController.a().onTrue(m_SetElevatorMiddle);
		}

		if (subsystemEnabled.getOrDefault("DRIVE", false)) {
			// DRIVE
			System.out.println("DRIVE - Subsystem enabled");
			m_driverController.x().onTrue(m_GoTo0Degrees);
			m_driverController.y().onTrue(m_GoTo90Degrees);
		}
	}

	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return Autos.exampleAuto(m_exampleSubsystem);
	}

}
