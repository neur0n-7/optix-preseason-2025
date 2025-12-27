package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.mechanism.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmConstants.ArmPositionStates;
import frc.robot.subsystems.arm.ArmConstants.CargoStates;
import frc.robot.subsystems.arm.ArmConstants.GripperStates;
import frc.robot.subsystems.arm.gripper.GripperIO;
import frc.robot.subsystems.arm.motor.ArmMotorIO;

public class ArmSubsystem extends SubsystemBase {

    // Motor + gripper
    private final ArmMotorIO armMotor;
    private final GripperIO gripper;

    // control
    private final ProfiledPIDController pidController;
    private final ArmFeedforward emptyFeedforward;
    private final ArmFeedforward loadedFeedforward;
    
    private double lastSetpointVelocity = 0.0;
    private double lastActualVelocity = 0.0;
    private double targetPositionDegrees = 0.0;
    
    // states
    private ArmPositionStates currentPositionState = ArmPositionStates.STOW;

    // mech
    private final LoggedMechanism2d mech = new LoggedMechanism2d(4, 4);
    private final LoggedMechanismLigament2d armMech;
    private final LoggedMechanismLigament2d gripperLeft;
    private final LoggedMechanismLigament2d gripperRight;
    


    public ArmSubsystem(ArmMotorIO motor, GripperIO gripper) {
        this.armMotor = motor;
        this.gripper = gripper;

        pidController = new ProfiledPIDController(
                ArmConstants.kP,
                ArmConstants.kI,
                ArmConstants.kD,
                new TrapezoidProfile.Constraints(
                    ArmConstants.maxEmptyVelocity,
                    ArmConstants.maxEmptyAccel
                )
        );

        pidController.setTolerance(ArmConstants.PIDTolerance);

        emptyFeedforward = new ArmFeedforward(
                ArmConstants.kS,
                ArmConstants.kGEmpty,
                ArmConstants.kVEmpty,
                ArmConstants.kAEmpty
        );

        loadedFeedforward = new ArmFeedforward(
            ArmConstants.kS,
            ArmConstants.kGLoaded,
            ArmConstants.kVLoaded,
            ArmConstants.kALoaded
        );

        LoggedMechanismRoot2d root = mech.getRoot("arm", 2, 2);
        armMech = root.append(
                new LoggedMechanismLigament2d(
                        "arm",
                        1.5,
                        0
                )
        );

        gripperLeft = armMech.append(
            new LoggedMechanismLigament2d(
                "gripperLeft",
                0.3,
                0
            )
        );

        gripperRight = armMech.append(
            new LoggedMechanismLigament2d(
                "gripperRight",
                0.3,
                0
            )
        );

        
    }

    // state setters
    public void setTargetPositionState(ArmPositionStates targetPositionState) {
        pidController.setGoal(targetPositionState.position_degs);
        targetPositionDegrees = targetPositionState.position_degs;
        currentPositionState = targetPositionState;
    }

    public void setGripperState(GripperStates targetGripperState){
        gripper.setGripperState(targetGripperState);
    }

    public void setCargoState(CargoStates targetCargoState){
        gripper.setCargoState(targetCargoState);
        armMotor.setArmMass(targetCargoState.isHoldingCone);
    }

    // getters
    public boolean atPositionTarget() {
        return pidController.atGoal();
    }

    public double getPositionDegrees() {
        return armMotor.getPositionDegrees();
    }

    // state getters
    public ArmPositionStates getPositionState() {
        return currentPositionState;
    }

    public GripperStates getGripperState() {
        return gripper.getGripperState();
    }

    public CargoStates getCargoState() {
        return gripper.getCargoState();
    }

    // IO stuff
    public void setMotorVoltage(double volts){
        armMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }

    public void updateMechanism(){
        double armAngleDegrees = getPositionDegrees();

        // 0 degrees in the mech points right, it should point down
        armMech.setAngle(armAngleDegrees - 90);

        double clawAngle = gripper.getGripperState() == GripperStates.OPEN ? 60.0 : 10.0;
        gripperLeft.setAngle(-clawAngle);
        gripperRight.setAngle(clawAngle);

    }

    @Override
    public void periodic() {
                
        double currentDegrees = getPositionDegrees();

        TrapezoidProfile.State setpoint = pidController.getSetpoint();

        double setpointVelocity = setpoint.velocity;
        double setpointAcceleration = (setpointVelocity - lastSetpointVelocity) / 0.02;
        lastSetpointVelocity = setpointVelocity;

        double actualVelocity = armMotor.getVelocity();
        double actualAcceleration = (actualVelocity - lastActualVelocity) / 0.02;
        lastActualVelocity = actualVelocity;

        double pidVolts = pidController.calculate(currentDegrees);

        double ffVolts;
        if (gripper.getCargoState().isHoldingCone){
            ffVolts = loadedFeedforward.calculate(Units.radiansToDegrees(getPositionDegrees()), setpointVelocity, setpointAcceleration);
        } else {
            ffVolts = emptyFeedforward.calculate(Units.radiansToDegrees(getPositionDegrees()), setpointVelocity, setpointAcceleration);
        }

        double totalVolts = MathUtil.clamp(pidVolts + ffVolts, -12, 12);
        
        setMotorVoltage(totalVolts);

        SmartDashboard.putNumber("Arm/Current Degrees", currentDegrees);
        SmartDashboard.putNumber("Arm/Target Degrees", targetPositionDegrees);

        SmartDashboard.putNumber("Arm/Volts PID", pidVolts);
        SmartDashboard.putNumber("Arm/Volts FF", ffVolts);
        SmartDashboard.putNumber("Arm/Volts TOTAL", totalVolts);
        SmartDashboard.putBoolean("Arm/At Target", atPositionTarget());

        SmartDashboard.putNumber("Arm/Velocity (Setpoint)", setpointVelocity);
        SmartDashboard.putNumber("Arm/Velocity (Actual)", actualVelocity);
        SmartDashboard.putNumber("Arm/Acceleration (Setpoint)", setpointAcceleration);
        SmartDashboard.putNumber("Arm/Acceleration (Actual)", actualAcceleration);

        SmartDashboard.putData("Arm/Mech2d", mech);

        SmartDashboard.putString("Arm/Position State", getPositionState().toString());
        SmartDashboard.putString("Arm/Gripper State", getGripperState().toString());
        SmartDashboard.putString("Arm/Cargo State", getCargoState().toString());

    }

    @Override
    public void simulationPeriodic(){
        armMotor.updateSimulation(0.02);
        updateMechanism();
    }
}
