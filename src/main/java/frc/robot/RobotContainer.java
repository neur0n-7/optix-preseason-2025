// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.drive.Drive;
import frc.robot.commands.drive.StopDriving;
import frc.robot.commands.elevator.GoToElevatorHeight;
import frc.robot.commands.swerve.JoystickDrive;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
// import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.RealNeoMotor;
import frc.robot.subsystems.SimNeoMotor;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...

	private final DriveSubsystem m_DriveSubsystem; // = new DriveSubsystem();
	private final Drive m_Drive;
	private final StopDriving m_StopDriving;

	private final ElevatorSubsystem m_ElevatorSubsystem; // = new ElevatorSubsystem();
	private final GoToElevatorHeight m_GoToElevatorHighest;
	private final GoToElevatorHeight m_GoToElevatorLowest;

	private final SwerveSubsystem m_SwerveSubsystem;
	private final JoystickDrive m_JoystickDrive;

	private final DoubleSupplier xSpeedSupplier;
	private final DoubleSupplier ySpeedSupplier;
	private final DoubleSupplier rotSpeedSupplier;
	private final BooleanSupplier fieldRelativeSupplier;

	private boolean fieldRelativeToggle;
	private boolean fieldRelativeLastState;

	private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController m_driverController =
		new CommandXboxController(OperatorConstants.kDriverControllerPort);

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();

		if (RobotBase.isSimulation()){
			m_DriveSubsystem = new DriveSubsystem(new SimNeoMotor());
			m_ElevatorSubsystem = new ElevatorSubsystem(new SimNeoMotor());
		} else {
			m_DriveSubsystem = new DriveSubsystem(new RealNeoMotor(OperatorConstants.driveMotorCanId));
			m_ElevatorSubsystem = new ElevatorSubsystem(new RealNeoMotor(OperatorConstants.elevatorMotorCanId));
		}

		m_GoToElevatorHighest = new GoToElevatorHeight(m_ElevatorSubsystem, ElevatorConstants.ElevatorStates.HIGHEST);
		m_GoToElevatorLowest = new GoToElevatorHeight(m_ElevatorSubsystem, ElevatorConstants.ElevatorStates.LOWEST);
	
		m_Drive = new Drive(m_DriveSubsystem);
		m_StopDriving = new StopDriving(m_DriveSubsystem);

		xSpeedSupplier = () -> -m_driverController.getLeftY();
		ySpeedSupplier = () -> -m_driverController.getLeftX();
		rotSpeedSupplier = () -> -m_driverController.getRightX();

		fieldRelativeToggle = false;
		fieldRelativeSupplier = () -> {
			if (m_driverController.b().getAsBoolean() && !fieldRelativeLastState) {
				fieldRelativeToggle = !fieldRelativeToggle;
			}
			fieldRelativeLastState = m_driverController.b().getAsBoolean();
			return fieldRelativeToggle;
		};

		fieldRelativeLastState = false;

		m_SwerveSubsystem = new SwerveSubsystem(RobotBase.isReal());
		m_JoystickDrive = new JoystickDrive(m_SwerveSubsystem, xSpeedSupplier, ySpeedSupplier, rotSpeedSupplier, fieldRelativeSupplier);
		m_SwerveSubsystem.setDefaultCommand(m_JoystickDrive);


	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
	 * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {
		m_driverController.a().whileTrue(m_Drive);
		m_driverController.a().whileFalse(m_StopDriving);

		m_driverController.x().onTrue(m_GoToElevatorHighest);
		m_driverController.y().onTrue(m_GoToElevatorLowest);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */

	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return Autos.exampleAuto(m_exampleSubsystem);
	}
  
}
