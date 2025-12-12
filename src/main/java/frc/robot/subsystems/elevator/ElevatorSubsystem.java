package frc.robot.subsystems.elevator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.NeoMotor;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;

public class ElevatorSubsystem extends SubsystemBase {

    private final NeoMotor motor;

    private double dist = 0.0;

    private ElevatorStates state = ElevatorStates.LOWEST;
    private double setpoint = 0.0; // in meters

    private Mechanism2d mechanism = new Mechanism2d(3, 3);
    private LoggedMechanismRoot2d root =  mechanism.getRoot("elevator", 1, 0);
    private LoggedMechanismLigament2d elevatorMech = root.append(
        new LoggedMechanismLigament2d("elevator", ElevatorConstants.elevatorBaseHeight, 90);
    )

    private MecahnismRoot2d elevatorRoot = elevator

    private ProfiledPIDController profiled = new ProfiledPIDController(
            ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD,
            new TrapezoidProfile.Constraints(
                ElevatorConstants.maxVelocity,
                ElevatorConstants.maxAccel
            )
    );

    private ElevatorFeedforward feedforward = new ElevatorFeedforward(
        ElevatorConstants.kS,
        ElevatorConstants.kG,
        ElevatorConstants.kV,
        ElevatorConstants.kA
    );

    public ElevatorSubsystem(NeoMotor motor) {
        profiled.setTolerance(0.02);

        this.motor = motor;

    }

    public void setTarget(double meters){
        setpoint = meters;
        profiled.setGoal(meters);
    }

    private double getHeight(){
        // TODO: double check this
        return motor.getPosition() * ElevatorConstants.gearing; 
    }

    public boolean atSetpoint(){ return profiled.atGoal(); }


    // used in sim only
    private void updateElevatorDist(double dist){
        this.dist = dist;
        mechanism.setLength(ElevatorConstants.elevatorBaseHeight + getHeight());
    }

    /*
     * commands shared by periodic() and simulationPeriodic()
    */
    public void execUpdates(boolean useFeedforward){
        double currentPosition = getHeight();

        double ffVolts = feedforward.calculate(
            profiled.getSetpoint().velocity,
            0
        );
        double pidOutput = profiled.calculate(currentPosition);

        double totalVolts = MathUtil.clamp(ffVolts + pidOutput, -12, 12);

        motor.setVoltage(totalVolts);

        // Logging
        SmartDashboard.putNumber("Elevator/Setpoint", setpoint);
        SmartDashboard.putNumber("Elevator/CurrentPos", currentPosition);
        SmartDashboard.putNumber("Elevator/Velocity", profiled.getSetpoint().velocity);
        SmartDashboard.putNumber("Elevator/PID Output (V)", pidOutput);
        SmartDashboard.putNumber("Elevator/FF Output (V)", ffVolts);
        SmartDashboard.putNumber("Elevator/Total Voltage", totalVolts);
        SmartDashboard.putString("Elevator/State", state.toString());
    }


    @Override
    public void periodic() {
        execUpdates(true);
    }

    @Override
    public void simulationPeriodic() {
        execUpdates(false);
        updateElevatorDist();
    }
}
