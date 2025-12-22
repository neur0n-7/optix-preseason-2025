// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
// import frc.robot.commands.elevatorV2.SetElevator12V;
import frc.robot.commands.elevatorV2.SetElevatorState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ExampleSubsystem;

// SWERVE
// import frc.robot.commands.swerve.JoystickDrive;
// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;
// import frc.robot.subsystems.swerve.SwerveSubsystem;

// DRIVE
// import frc.robot.subsystems.drive.DriveSubsystem;
// import frc.robot.commands.drive.GoToDegrees;

// ELEVATOR V1
// import frc.robot.commands.elevator.GoToElevatorHeight;
// import frc.robot.subsystems.elevator.ElevatorConstants;
// import frc.robot.subsystems.elevator.ElevatorSubsystem;

import frc.robot.subsystems.RealNeoMotor;
import frc.robot.subsystems.SimNeoMotor;

// ELEVATOR V2
import frc.robot.subsystems.elevatorV2.V2ElevatorConstants;
import frc.robot.subsystems.elevatorV2.V2ElevatorSubsystem;

public class RobotContainer {

	// DRIVE
	// private final DriveSubsystem m_DriveSubsystem;
	// private final GoToDegrees m_GoTo90Degrees;
	// private final GoToDegrees m_GoTo0Degrees;

	// ELEVATOR V1
	// private final ElevatorSubsystem m_ElevatorSubsystem;
	// private final GoToElevatorHeight m_GoToElevatorHighest;
	// private final GoToElevatorHeight m_GoToElevatorLowest;
	// private final GoToElevatorHeight m_GoToElevatorMiddle;

	// SWERVE
	// private final SwerveSubsystem m_SwerveSubsystem;
	// private final JoystickDrive m_JoystickDrive;
	// private final DoubleSupplier xSpeedSupplier;
	// private final DoubleSupplier ySpeedSupplier;
	// private final DoubleSupplier rotSpeedSupplier;
	// private final BooleanSupplier fieldRelativeSupplier;
	// private boolean fieldRelativeToggle;
	// private boolean fieldRelativeLastState;

	// ELEVATOR V2
	private final V2ElevatorSubsystem m_ElevatorSubsystem2;
	private final SetElevatorState m_GoToElevatorHighest;
	private final SetElevatorState m_GoToElevatorLowest;
	private final SetElevatorState m_GoToElevatorMiddle;

	// private final SetElevator12V m_SetElevator12V;

	

	private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

	private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

	public RobotContainer() {
		
		if (RobotBase.isSimulation()){
			// DRIVE
			// m_DriveSubsystem = new DriveSubsystem(new SimNeoMotor());

			// ELEVATOR V1
			// m_ElevatorSubsystem = new ElevatorSubsystem(new SimNeoMotor(), false);

			// ELEVATOR V2
			m_ElevatorSubsystem2 = new V2ElevatorSubsystem(new SimNeoMotor(), true);
			
			
		} else {
			// DRIVE
			// m_DriveSubsystem = new DriveSubsystem(new RealNeoMotor(OperatorConstants.driveMotorCanId));

			// ELEVATOR V1
			// m_ElevatorSubsystem = new ElevatorSubsystem(new RealNeoMotor(OperatorConstants.elevatorMotorCanId), true);

			// ELEVATOR V2
			m_ElevatorSubsystem2 = new V2ElevatorSubsystem(new RealNeoMotor(OperatorConstants.elevatorMotorCanId), false);
		}

		// ELEVATOR V1 ////////////////////
		// m_GoToElevatorHighest = new GoToElevatorHeight(m_ElevatorSubsystem, ElevatorConstants.ElevatorStates.HIGHEST);
		// m_GoToElevatorLowest = new GoToElevatorHeight(m_ElevatorSubsystem, ElevatorConstants.ElevatorStates.LOWEST);
		// m_GoToElevatorMiddle = new GoToElevatorHeight(m_ElevatorSubsystem, ElevatorConstants.ElevatorStates.MIDDLE);

		// ELEVATOR V2 ////////////////////
		m_GoToElevatorLowest = new SetElevatorState(m_ElevatorSubsystem2, V2ElevatorConstants.ElevatorStates.LOWEST);
		m_GoToElevatorMiddle = new SetElevatorState(m_ElevatorSubsystem2, V2ElevatorConstants.ElevatorStates.MIDDLE);
		m_GoToElevatorHighest = new SetElevatorState(m_ElevatorSubsystem2, V2ElevatorConstants.ElevatorStates.HIGHEST);

		// m_SetElevator12V = new SetElevator12V(m_ElevatorSubsystem2);

		// DRIVE ////////////////////
		// m_GoTo90Degrees = new GoToDegrees(m_DriveSubsystem, 90);
		// m_GoTo0Degrees = new GoToDegrees(m_DriveSubsystem, 0);

		// SWERVE ////////////////////

		// Supplier lambdas
		// left joystick handles movement, right joystick handles turning
		// xSpeedSupplier = () -> -m_driverController.getLeftY();
		// ySpeedSupplier = () -> m_driverController.getLeftX();
		// rotSpeedSupplier = () -> m_driverController.getRightX();

		// fieldRelativeToggle = false;

		/*
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
		/* 
		

		// CONFIGURE BINDINGS ////////////////////
		configureBindings();

	}

	private void configureBindings() {
		/*
		 * The bindings are as follows:
		 *  - Left joystick to move robot w/ swerve
		 *  - Right joystick to rotate robot w/ swerve
		 * 
		 *  - X to go to elevator max
		 *  - Y to go to elevator min
		 *  - A to go to elevator mid
		 * 
		 * 	Drive (single motor):
		 *  - Left bumper to go to 0 degrees
		 *  - Right bumper to go to 90 degrees
		 */

		// SWERVE
		// m_SwerveSubsystem.setDefaultCommand(m_JoystickDrive);

		// ELEVATOR V1
		// m_driverController.x().onTrue(m_GoToElevatorHighest);
		// m_driverController.y().onTrue(m_GoToElevatorLowest);
		// m_driverController.a().onTrue(m_GoToElevatorMiddle);

		// ELEVATOR V2
		m_driverController.x().onTrue(m_GoToElevatorHighest);
		m_driverController.y().onTrue(m_GoToElevatorLowest);
		m_driverController.a().onTrue(m_GoToElevatorMiddle);
		// m_driverController.b().onTrue(m_SetElevator12V);
		 
		// DRIVE
		// m_driverController.leftBumper().onTrue(m_GoTo0Degrees);
		// m_driverController.rightBumper().onTrue(m_GoTo90Degrees);
		
	}

	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return Autos.exampleAuto(m_exampleSubsystem);
	}
  
}
