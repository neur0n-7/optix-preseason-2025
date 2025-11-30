package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.NeoMotor;

public class DriveSubsystem extends SubsystemBase {

	private final NeoMotor motor; // new SparkMax(22, MotorType.kBrushless);
    private final PIDController pid = new PIDController(0.1, 0.0, 0.0);

    private double setpoint = 0.0;
	private double simulatedVoltage = 0.0;

	public DriveSubsystem(NeoMotor motor) {
		pid.setTolerance(0.05);
		this.motor = motor;

	}

	public void setMotorVoltage(double volts){
		motor.setVoltage(volts);
	}

	public void setMotorVoltagePID(double volts){
		setpoint = volts;
	}

  @Override
  public void periodic() {
	// This method will be called once per scheduler run

	// Update PID
	double currentVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
	double output = pid.calculate(currentVoltage, setpoint);

	// clamp from -12 to 12 v
	output = Math.max(Math.min(output, 12.0), -12.0);

	motor.setVoltage(output);

	simulatedVoltage = output;

	// logging

	SmartDashboard.putNumber("Drive/Setpoint", setpoint);
	SmartDashboard.putNumber("Drive/ActualVoltage", currentVoltage);
	SmartDashboard.putNumber("Drive/PIDOutputVoltage", output);
	SmartDashboard.putNumber("Drive/SimulatedVoltage", simulatedVoltage);

  }

  @Override
  public void simulationPeriodic() {
	// This method will be called once per scheduler run during simulation
	double applied = motor.getAppliedOutput() * 12.0;
	simulatedVoltage += (applied - simulatedVoltage) * 0.1;
	SmartDashboard.putNumber("Drive/Sim Voltage (sim)", simulatedVoltage);

  }
}
