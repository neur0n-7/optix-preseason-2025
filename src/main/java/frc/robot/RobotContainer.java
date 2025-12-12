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
// import frc.robot.commands.drive.Drive;
// import frc.robot.commands.drive.StopDriving;
// import frc.robot.commands.driveV2.GoToDegrees;
import frc.robot.commands.elevator.GoToElevatorHeight;
import frc.robot.commands.swerve.JoystickDrive;
// import frc.robot.subsystems.drive.DriveSubsystem;
// import frc.robot.subsystems.driveV2.DriveSubsystemV2;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
// import frc.robot.subsystems.NeoMotor;
import frc.robot.subsystems.RealNeoMotor;
import frc.robot.subsystems.SimNeoMotor;


public class RobotContainer {

	
	// private final DriveSubsystem m_DriveSubsystem;
	// private final Drive m_Drive;
	// private final StopDriving m_StopDriving;

	private final ElevatorSubsystem m_ElevatorSubsystem;
	private final GoToElevatorHeight m_GoToElevatorHighest;
	private final GoToElevatorHeight m_GoToElevatorLowest;
	private final GoToElevatorHeight m_GoToElevatorMiddle;

	private final SwerveSubsystem m_SwerveSubsystem;
	private final JoystickDrive m_JoystickDrive;

	private final DoubleSupplier xSpeedSupplier;
	private final DoubleSupplier ySpeedSupplier;
	private final DoubleSupplier rotSpeedSupplier;
	private final BooleanSupplier fieldRelativeSupplier;

	private boolean fieldRelativeToggle;
	private boolean fieldRelativeLastState;
	

	private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
	// private final DriveSubsystemV2 m_DriveSubsystemV2;
	// private final GoToDegrees m_DriveGoTo90;
	// private final GoToDegrees m_DriveGoTo0;


	private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

	public RobotContainer() {

		configureBindings();
		
		if (RobotBase.isSimulation()){
			// m_DriveSubsystemV2 = new DriveSubsystemV2(new SimNeoMotor());
			// m_DriveSubsystem = new DriveSubsystem(new SimNeoMotor());
			m_ElevatorSubsystem = new ElevatorSubsystem(new SimNeoMotor());
		} else {
			// m_DriveSubsystemV2 = new DriveSubsystemV2(new RealNeoMotor(OperatorConstants.driveMotorCanId));
			// m_DriveSubsystem = new DriveSubsystem(new RealNeoMotor(OperatorConstants.driveMotorCanId));
			m_ElevatorSubsystem = new ElevatorSubsystem(new RealNeoMotor(OperatorConstants.elevatorMotorCanId));
		}

		// DRIVE (v2)
		// m_DriveGoTo90 = new GoToDegrees(m_DriveSubsystemV2, 90);
		// m_DriveGoTo0 = new GoToDegrees(m_DriveSubsystemV2, 0);

		// ELEVATOR
		m_GoToElevatorHighest = new GoToElevatorHeight(m_ElevatorSubsystem, ElevatorConstants.ElevatorStates.HIGHEST);
		m_GoToElevatorLowest = new GoToElevatorHeight(m_ElevatorSubsystem, ElevatorConstants.ElevatorStates.LOWEST);
		m_GoToElevatorMiddle = new GoToElevatorHeight(m_ElevatorSubsystem, ElevatorConstants.ElevatorStates.MIDDLE);

		// DRIVE (v1)
		// m_Drive = new Drive(m_DriveSubsystem);
		// m_StopDriving = new StopDriving(m_DriveSubsystem);

		// SWERVE
		// left joystick handles movement, right joystick handles turning
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
		m_JoystickDrive = new JoystickDrive(
			m_SwerveSubsystem,
			xSpeedSupplier,
			ySpeedSupplier,
			rotSpeedSupplier,
			fieldRelativeSupplier
		);	
	}

	private void configureBindings() {
		
		// SWERVE
		m_SwerveSubsystem.setDefaultCommand(m_JoystickDrive);

		// ELEVATOR
		m_driverController.x().onTrue(m_GoToElevatorHighest);
		m_driverController.y().onTrue(m_GoToElevatorLowest);
		m_driverController.a().onTrue(m_GoToElevatorMiddle);
		 
		// DRIVE (v1)
		// m_driverController.a().onTrue(m_Drive);
		// m_driverController.a().onFalse(m_StopDriving);

		// DRIVE (v2)
		// m_driverController.x().onFalse(m_DriveGoTo90);
		// m_driverController.x().onTrue(m_DriveGoTo0);		 
		
	}


	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return Autos.exampleAuto(m_exampleSubsystem);
	}
  
}
