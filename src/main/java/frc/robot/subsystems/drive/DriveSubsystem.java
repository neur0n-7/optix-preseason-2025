package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;

public class DriveSubsystem extends SubsystemBase {

    private static LoggedTunableNumber kPLogger = new LoggedTunableNumber("Drive/kP", DriveConstants.kP);


	private final NeoMotorIO motor; // new SparkMax(22, MotorType.kBrushless);
	private final PIDController pid = new PIDController(
			kPLogger.get(),
			DriveConstants.kI,
			DriveConstants.kD);

	private double targetDegrees = 0.0;

	public DriveSubsystem(NeoMotorIO motor) {
		pid.setTolerance(DriveConstants.PIDTolerance);
		this.motor = motor;
	}

	public void setTargetDegrees(double degrees) {
		targetDegrees = degrees;
		pid.reset();
	}


	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		double currentVoltage = motor.getAppliedOutput() * motor.getBusVoltage();

		double currentDegrees = motor.getPositionMeters();

		pid.setP(kPLogger.get());
		double output = pid.calculate(currentDegrees, targetDegrees);

		output = MathUtil.clamp(output, -12.0, 12.0);
		// output = 5;

		motor.setVoltage(output);

		// logging
		SmartDashboard.putNumber("Drive/TargetDegs", targetDegrees);
		SmartDashboard.putNumber("Drive/CurrentDegs", currentDegrees);
		SmartDashboard.putNumber("Drive/ActualVoltage", currentVoltage);
		SmartDashboard.putNumber("Drive/PIDOutputVoltage", output);
		SmartDashboard.putBoolean("Drive/At Setpoint", pid.atSetpoint());

		Logger.recordOutput("Drive/CurrentDegs", currentDegrees);
		Logger.recordOutput("Drive/TargetDegs", targetDegrees);

	}

	@Override
	public void simulationPeriodic() {
		motor.updateSimulation(0.02);
	}
}
