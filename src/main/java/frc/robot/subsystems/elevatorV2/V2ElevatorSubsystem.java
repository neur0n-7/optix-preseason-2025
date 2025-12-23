package frc.robot.subsystems.elevatorV2;

import org.littletonrobotics.junction.mechanism.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevatorV2.V2ElevatorConstants.ElevatorStates;

public class V2ElevatorSubsystem extends SubsystemBase {

    private final SimElevatorMotor motor;

    private final ProfiledPIDController pidController;
    private final ElevatorFeedforward feedforward;

    private double lastSetpointVelocity = 0.0;

    private double lastActualHeight = 0.0;
    private double lastActualVelocity = 0.0;

    private double targetMeters = 0.0;

    private ElevatorStates state = ElevatorStates.LOWEST;

    private final LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);
    private final LoggedMechanismLigament2d elevatorMech;

    public V2ElevatorSubsystem(SimElevatorMotor motor, boolean isSim) {
        this.motor = motor;

        pidController = new ProfiledPIDController(
                V2ElevatorConstants.kP,
                V2ElevatorConstants.kI,
                V2ElevatorConstants.kD,
                new TrapezoidProfile.Constraints(
                        V2ElevatorConstants.maxVelocity,
                        V2ElevatorConstants.maxAccel));
        pidController.setTolerance(0.01);

        if (isSim) {
            feedforward = new ElevatorFeedforward(0.0, 0.0, V2ElevatorConstants.kV, V2ElevatorConstants.kA);
        } else {
            feedforward = new ElevatorFeedforward(
                    V2ElevatorConstants.kS,
                    V2ElevatorConstants.kG,
                    V2ElevatorConstants.kV,
                    V2ElevatorConstants.kA);
        }

        LoggedMechanismRoot2d root = mech.getRoot("elevator", 1, 0);
        elevatorMech = root.append(
                new LoggedMechanismLigament2d(
                        "elevator",
                        0.0,
                        90));
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
        return motor.getPosition() * V2ElevatorConstants.metersPerMotorRotation;
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

        double actualVelocity = (currentPosition - lastActualHeight) / 0.02;
        double actualAccel = (actualVelocity - lastActualVelocity) / 0.02;

        lastActualHeight = currentPosition;
        lastActualVelocity = actualVelocity;

        double ffVolts = feedforward.calculate(setpointVelocity, setpointAcceleration);
        // double ffVolts = 0.0;

        double outputVolts = MathUtil.clamp(pidVolts + ffVolts, -12.0, 12.0);
        setMotorVoltage(outputVolts);

        elevatorMech.setLength(currentPosition);

        SmartDashboard.putNumber("ElevatorV2/Position", currentPosition);
        SmartDashboard.putNumber("ElevatorV2/Target", targetMeters);
        SmartDashboard.putString("ElevatorV2/Target State", state.toString());
        // SmartDashboard.putNumber("ElevatorV2/Error", targetMeters - currentPosition);

        boolean isAtTarget = atTarget();

        if (isAtTarget) {
            SmartDashboard.putString("ElevatorV2/Action", "HOLD %s".formatted(state.toString()));
        } else {
            if (setpointAcceleration > 0) {
                SmartDashboard.putString("ElevatorV2/Action", "ACCEL");
            } else if (setpointAcceleration < 0) {
                SmartDashboard.putString("ElevatorV2/Action", "DECEL");
            } else {
                SmartDashboard.putString("ElevatorV2/Action", "CRUISE");
            }
        }

        SmartDashboard.putNumber("ElevatorV2/Velocity (Setpoint)", setpointVelocity);
        SmartDashboard.putNumber("ElevatorV2/Velocity (Actual)", actualVelocity);

        SmartDashboard.putNumber("ElevatorV2/Acceleration (Setpoint)", setpointAcceleration);
        SmartDashboard.putNumber("ElevatorV2/Acceleration (Actual)", actualAccel);

        SmartDashboard.putNumber("ElevatorV2/PID Volts", pidVolts);
        SmartDashboard.putNumber("ElevatorV2/FF Volts", ffVolts);
        SmartDashboard.putNumber("ElevatorV2/Total Volts", outputVolts);
        SmartDashboard.putBoolean("ElevatorV2/At Target", isAtTarget);
        SmartDashboard.putData("ElevatorV2/Mech2d", mech);

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
