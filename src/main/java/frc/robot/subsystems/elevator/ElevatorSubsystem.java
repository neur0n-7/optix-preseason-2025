package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.mechanism.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;

public class ElevatorSubsystem extends SubsystemBase {

    private final ElevatorMotorIO motor;

    private final ProfiledPIDController pidController;
    private final ElevatorFeedforward feedforward;

    private double lastSetpointVelocity = 0.0;

    private double lastActualVelocity = 0.0;

    private double targetMeters = 0.0;

    private ElevatorStates state = ElevatorStates.LOWEST;

    private final LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);
    private final LoggedMechanismLigament2d elevatorMech;

    public ElevatorSubsystem(ElevatorMotorIO motor) {
        this.motor = motor;

        pidController = new ProfiledPIDController(
                ElevatorConstants.kP,
                ElevatorConstants.kI,
                ElevatorConstants.kD,
                new TrapezoidProfile.Constraints(
                        ElevatorConstants.maxVelocity,
                        ElevatorConstants.maxAccel
                )
        );

        pidController.setTolerance(0.01);

        feedforward = new ElevatorFeedforward(
                ElevatorConstants.kS,
                ElevatorConstants.kG,
                ElevatorConstants.kV,
                ElevatorConstants.kA
        );

        LoggedMechanismRoot2d root = mech.getRoot("elevator", 1, 0);
        elevatorMech = root.append(
                new LoggedMechanismLigament2d(
                        "elevator",
                        0.0,
                        90
                )
        );
    }

    public void setTargetState(ElevatorStates targetState) {
        pidController.setGoal(targetState.position);
        targetMeters = targetState.position;
        state = targetState;
    }

    public boolean atTarget() {
        return pidController.atGoal();
    }

    // meters
    public double getHeight() {
        return motor.getPositionMeters();
    }

    public void setMotorVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void periodic() {
        double currentPosition = getHeight();

        double pidVolts = pidController.calculate(currentPosition);

        TrapezoidProfile.State setpoint = pidController.getSetpoint();

        double setpointVelocity = setpoint.velocity;
        double setpointAcceleration = (setpointVelocity - lastSetpointVelocity) / 0.02;
        lastSetpointVelocity = setpointVelocity;

        double actualVelocity = motor.getVelocity();
        double actualAccel = (actualVelocity - lastActualVelocity) / 0.02;

        lastActualVelocity = actualVelocity;

        double ffVolts = feedforward.calculate(setpointVelocity, setpointAcceleration);

        double totalVolts = MathUtil.clamp(pidVolts + ffVolts, -12.0, 12.0);

        setMotorVoltage(totalVolts);

        elevatorMech.setLength(currentPosition);

        SmartDashboard.putNumber("Elevator/Position", currentPosition);
        SmartDashboard.putNumber("Elevator/Target", targetMeters);
        SmartDashboard.putString("Elevator/Target State", state.toString());
        SmartDashboard.putNumber("Elevator/Error", targetMeters - currentPosition);

        SmartDashboard.putNumber("Elevator/Velocity (Setpoint)", setpointVelocity);
        SmartDashboard.putNumber("Elevator/Velocity (Actual)", actualVelocity);

        SmartDashboard.putNumber("Elevator/Acceleration (Setpoint)", setpointAcceleration);
        SmartDashboard.putNumber("Elevator/Acceleration (Actual)", actualAccel);

        SmartDashboard.putNumber("Elevator/PID Volts", pidVolts);
        SmartDashboard.putNumber("Elevator/FF Volts", ffVolts);
        SmartDashboard.putNumber("Elevator/Total Volts", totalVolts);
        SmartDashboard.putBoolean("Elevator/At Target", atTarget());
        SmartDashboard.putData("Elevator/Mech2d", mech);

        /*
         * System.out.println("PERIODIC CALLED");
         * System.out.println(
         * "POS %f, SETPT %f, VELOCITY %f, ACCEL %f, PID V %f, FF V %f".formatted(
         * position, setpointMeters, currentVelocity, acceleration, pidVolts, ffVolts)
         * );
         */

    }

    @Override
    public void simulationPeriodic() {
        motor.updateSimulation(0.02);
    }
}
