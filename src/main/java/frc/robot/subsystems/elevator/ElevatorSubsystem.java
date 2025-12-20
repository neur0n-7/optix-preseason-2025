package frc.robot.subsystems.elevator;
// import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.NeoMotor;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;

public class ElevatorSubsystem extends SubsystemBase {

    private final NeoMotor motor;

    private ElevatorStates state = ElevatorStates.LOWEST;
    private double setpoint = 0.0; // in meters

    private double lastHeight = 0.0; // in order to calculate velociyt
    private double lastVelocity = 0.0; // in order to calculate acceleration


    private boolean useFeedforward;

    private LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);
    private LoggedMechanismRoot2d root =  mech.getRoot("elevator", 1, 0);
    private LoggedMechanismLigament2d elevatorMech = root.append(
        new LoggedMechanismLigament2d("elevator", ElevatorConstants.elevatorBaseHeight, 90)
    );

    private ProfiledPIDController profiled = new ProfiledPIDController(
            ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD,
            new TrapezoidProfile.Constraints(
                ElevatorConstants.maxVelocity,
                ElevatorConstants.maxAccel
            )
    );

    private ElevatorFeedforward feedforward;

    public ElevatorSubsystem(NeoMotor motor, boolean isSim) {
        profiled.setTolerance(0.1);

        this.motor = motor;

        if (isSim){
            this.feedforward = new ElevatorFeedforward(
                // gravity + friction aren't there in sim
                0,
                0,
                ElevatorConstants.kV,
                ElevatorConstants.kA
            );
        } else {
            this.feedforward = new ElevatorFeedforward(
                ElevatorConstants.kS,
                ElevatorConstants.kG,
                ElevatorConstants.kV,
                ElevatorConstants.kA
            );
        }

    }

    public void setTarget(double meters){
        setpoint = meters;
        profiled.setGoal(meters);
    }

    private double getHeight(){
        double motorRots = motor.getPosition();
        return motorRots * ElevatorConstants.metersPerMotorRotation;
    }


    public boolean atSetpoint(){ return profiled.atGoal(); }

    private void updateElevatorDist(){
        double dist = getHeight();
        elevatorMech.setLength(ElevatorConstants.elevatorBaseHeight + dist);
    }

    @Override
    public void periodic() {
        double currentHeight = getHeight();

        State setpointState = profiled.getSetpoint();

        double currentVelocity = setpointState.velocity;
        double currentAccel = (currentVelocity - lastVelocity) / 0.02;
        lastVelocity = currentVelocity;

        double ffVolts = feedforward.calculate(
            currentVelocity,
            currentAccel
        );
        double pidOutput = profiled.calculate(currentHeight);

        double totalVolts = MathUtil.clamp(ffVolts + pidOutput, -12, 12);
    

        motor.setVoltage(totalVolts);

        // Logging
        SmartDashboard.putNumber("Elevator/Setpoint", setpoint);
        SmartDashboard.putNumber("Elevator/CurrentHeight", currentHeight);
        SmartDashboard.putNumber("Elevator/Velocity", currentVelocity);
        SmartDashboard.putNumber("Elevator/Acceleration", currentAccel);
        

        SmartDashboard.putNumber("Elevator/PID Output (V)", pidOutput);
        SmartDashboard.putNumber("Elevator/FF Output (V)", ffVolts);
        SmartDashboard.putNumber("Elevator/Total Voltage", totalVolts);

        SmartDashboard.putString("Elevator/State", state.toString());
        SmartDashboard.putBoolean("Elevator/AtSetpoint", profiled.atGoal());
    }

    @Override
    public void simulationPeriodic() {
        motor.updateSimulation(0.02);
        updateElevatorDist();
    }
}
