package frc.robot.subsystems.elevatorV2;

import org.littletonrobotics.junction.mechanism.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.NeoMotor;
import frc.robot.subsystems.elevatorV2.V2ElevatorConstants.ElevatorStates;

public class V2ElevatorSubsystem extends SubsystemBase {


    private final NeoMotor motor;

    private final ProfiledPIDController pidController;
    private final ElevatorFeedforward feedforward;

    private double lastVelocity = 0.0;

    private double setpointMeters = 0.0;

    private ElevatorStates state = ElevatorStates.LOWEST;

    private final LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);
    private final LoggedMechanismLigament2d elevatorMech;

    public V2ElevatorSubsystem(NeoMotor motor, boolean isSim) {
        this.motor = motor;

        pidController = new ProfiledPIDController(
            V2ElevatorConstants.kP,
            V2ElevatorConstants.kI,
            V2ElevatorConstants.kD,
            new TrapezoidProfile.Constraints(
                V2ElevatorConstants.maxVelocity,
                V2ElevatorConstants.maxAccel
            )
        );
        pidController.setTolerance(0.01);


        if (isSim){
            feedforward = new ElevatorFeedforward(0.0, 0.0, V2ElevatorConstants.kV, V2ElevatorConstants.kA);
        } else {
            feedforward = new ElevatorFeedforward(
                V2ElevatorConstants.kS,
                V2ElevatorConstants.kG,
                V2ElevatorConstants.kV,
                V2ElevatorConstants.kA
            );
        }

        LoggedMechanismRoot2d root = mech.getRoot("elevator", 1, 0);
        elevatorMech = root.append(
            new LoggedMechanismLigament2d(
                "elevator",
                V2ElevatorConstants.elevatorBaseHeight,
                90
            )
        );
    }

    public void setTargetState(ElevatorStates targetState){
        pidController.setGoal(targetState.position);
        setpointMeters = targetState.position;
        state = targetState;
    }

    public boolean atTarget() {
        return pidController.atGoal();
    }

    // meters
    public double getHeight() {
        return motor.getPosition() * V2ElevatorConstants.metersPerMotorRotation;
    }


    @Override
    public void periodic() {
        double position = getHeight();

        double pidVolts = pidController.calculate(position);

        TrapezoidProfile.State setpoint = pidController.getSetpoint();

        double currentVelocity = setpoint.velocity;
        double acceleration = (currentVelocity - lastVelocity) / 0.02;
        
        lastVelocity = currentVelocity;

        double ffVolts = feedforward.calculate(currentVelocity, acceleration);

        double outputVolts = MathUtil.clamp(pidVolts + ffVolts, -12.0, 12.0);
        motor.setVoltage(outputVolts);

        elevatorMech.setLength(V2ElevatorConstants.elevatorBaseHeight + position);

        SmartDashboard.putNumber("ElevatorV2/Position", position);
        SmartDashboard.putNumber("ElevatorV2/Setpoint", setpointMeters);
        SmartDashboard.putString("ElevatorV2/Target State", state.toString());
        SmartDashboard.putNumber("ElevatorV2/Error", setpointMeters - position);

        SmartDashboard.putNumber("ElevatorV2/Velocity", currentVelocity);
        SmartDashboard.putNumber("ElevatorV2/Acceleration", acceleration);

        SmartDashboard.putNumber("ElevatorV2/PID Volts", pidVolts);
        SmartDashboard.putNumber("ElevatorV2/FF Volts", ffVolts);
        SmartDashboard.putNumber("ElevatorV2/Total Volts", outputVolts);
        SmartDashboard.putBoolean("ElevatorV2/At Target", atTarget());
        SmartDashboard.putData("ElevatorV2/Mech2d", mech);

    
    }

    @Override
    public void simulationPeriodic() {
        motor.updateSimulation(0.02);
    }
}
