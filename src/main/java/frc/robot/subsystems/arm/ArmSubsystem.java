package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
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
    private double lastActualVelocityRads = 0.0;
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
                    ArmConstants.maxEmptyVelocityRads,
                    ArmConstants.maxEmptyAccelRads
                )
        );

        lastSetpointVelocity = pidController.getSetpoint().velocity;


        pidController.setTolerance(Units.degreesToRadians(ArmConstants.PIDToleranceDegrees));

        // FF for arm when not holding cone
        emptyFeedforward = new ArmFeedforward(
                ArmConstants.kS,
                ArmConstants.kGEmpty,
                ArmConstants.kVEmpty,
                ArmConstants.kAEmpty
        );

        // FF for arm when holding cone
        loadedFeedforward = new ArmFeedforward(
            ArmConstants.kS,
            ArmConstants.kGLoaded,
            ArmConstants.kVLoaded,
            ArmConstants.kALoaded
        );

        LoggedMechanismRoot2d root = mech.getRoot("arm", 2, 2);

        Color8Bit orange = new Color8Bit(255, 165, 0);

        armMech = root.append(
                new LoggedMechanismLigament2d(
                        "arm",
                        1.5,
                        0,
                        12,
                        orange
                )
        );

        gripperLeft = armMech.append(
            new LoggedMechanismLigament2d(
                "gripperLeft",
                0.3,
                0,
                12,
                orange
            )
        );

        gripperRight = armMech.append(
            new LoggedMechanismLigament2d(
                "gripperRight",
                0.3,
                0,
                12,
                orange
            )
        ); 
    }

    // Set the target POSITION state of the arm e.g. ArmPositionStates.STOW
    public void setTargetPositionState(ArmPositionStates targetPositionState) {
        pidController.setGoal(Units.degreesToRadians(targetPositionState.position_degs));
        targetPositionDegrees = targetPositionState.position_degs;
        currentPositionState = targetPositionState;
    }

    // Set the gripper state to open/closed
    public void setGripperState(GripperStates gripperState){
        gripper.setGripperState(gripperState);
    }

    // Set the cargo state to empty/loaded
    public void setCargoState(CargoStates cargoState){
        gripper.setCargoState(cargoState);
        armMotor.setSimArmMass(cargoState.isHoldingCone);
    }

    // Return whether or not the arm is at the target position (degrees)
    public boolean atPositionTarget() {
        return pidController.atGoal();
    }

    // Get the current position of the arm in degrees
    public double getPositionDegrees() {
        return armMotor.getPositionDegrees();
    }

    // State getters
    public ArmPositionStates getPositionState() {
        return currentPositionState;
    }

    public GripperStates getGripperState() {
        return gripper.getGripperState();
    }

    public CargoStates getCargoState() {
        return gripper.getCargoState();
    }

    // Set the voltage of the arm motor
    public void setMotorVoltage(double volts){
        armMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }

    @Override
    public void periodic() {
                
        TrapezoidProfile.State setpoint = pidController.getSetpoint();

        // Setpoint velocity/acceleration
        double setpointVelocity = setpoint.velocity;
        double setpointAcceleration = (setpointVelocity - lastSetpointVelocity) / 0.02;
        lastSetpointVelocity = setpointVelocity;

        // Actual velocity/acceleration
        boolean holdingCone = gripper.getCargoState().isHoldingCone;
        double maxAccel = holdingCone ? ArmConstants.maxLoadedAccelRads : ArmConstants.maxEmptyAccelRads;
        double maxVel   = holdingCone ? ArmConstants.maxLoadedVelocityRads : ArmConstants.maxEmptyVelocityRads;
        double actualVelocityRads = MathUtil.clamp(armMotor.getVelocityRadPerSec(), -maxVel, maxVel);
        double actualAccelerationRads = MathUtil.clamp((actualVelocityRads - lastActualVelocityRads) / 0.02, -maxAccel, maxAccel);
        lastActualVelocityRads = actualVelocityRads;

        // PID
        double currentDegrees = getPositionDegrees();
        double currentRadians = Units.degreesToRadians(currentDegrees);
        double pidVolts = MathUtil.clamp(pidController.calculate(currentRadians), -12, 12);

        // FF
        double ffVolts;
        if (holdingCone){
            ffVolts = loadedFeedforward.calculate(Units.degreesToRadians(getPositionDegrees() - 90), setpointVelocity, setpointAcceleration);
        } else {
            ffVolts = emptyFeedforward.calculate(Units.degreesToRadians(getPositionDegrees() - 90), setpointVelocity, setpointAcceleration);
        }
        ffVolts = MathUtil.clamp(ffVolts, -12, 12);


        // Send total voltage to arm motor
        double totalVolts = MathUtil.clamp(pidVolts + ffVolts, -12, 12);
        setMotorVoltage(totalVolts);

        SmartDashboard.putNumber("Arm/Current Degrees", currentDegrees);
        SmartDashboard.putNumber("Arm/Target Degrees", targetPositionDegrees);

        SmartDashboard.putNumber("Arm/Volts PID", pidVolts);
        SmartDashboard.putNumber("Arm/Volts FF", ffVolts);
        SmartDashboard.putNumber("Arm/Volts TOTAL", totalVolts);
        SmartDashboard.putBoolean("Arm/At Target", atPositionTarget());

        SmartDashboard.putNumber("Arm/Velocity (Setpoint)", setpointVelocity);
        SmartDashboard.putNumber("Arm/Velocity (Actual)", actualVelocityRads);
        SmartDashboard.putNumber("Arm/Acceleration (Setpoint)", setpointAcceleration);
        SmartDashboard.putNumber("Arm/Acceleration (Actual)", actualAccelerationRads);

        SmartDashboard.putData("Arm/Mech2d", mech);

        SmartDashboard.putString("Arm/Position State", getPositionState().toString());
        SmartDashboard.putString("Arm/Gripper State", getGripperState().toString());
        SmartDashboard.putString("Arm/Cargo State", getCargoState().toString());
    }

    // Update the LoggedMechanism2d
    public void updateMechanism(){
        double armAngleDegrees = getPositionDegrees();

        // Currently angles are stored with 0 degrees pointing down.
        // However, 0 degrees in Mechanism2ds points right
        armMech.setAngle(armAngleDegrees - 90);

        double clawAngle = gripper.getGripperState() == GripperStates.OPEN ? 60.0 : 10.0;
        gripperLeft.setAngle(-clawAngle);
        gripperRight.setAngle(clawAngle);

    }

    @Override
    public void simulationPeriodic(){
        armMotor.updateSimulation(0.02);
        updateMechanism();
    }
}
