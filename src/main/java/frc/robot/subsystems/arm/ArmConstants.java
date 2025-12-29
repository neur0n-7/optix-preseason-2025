package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {

    // Constraints
    public static final double maxEmptyVelocityRads = 5.387;
    public static final double maxEmptyAccelRads = 1.746;
    
    public static final double maxLoadedVelocityRads = 5.469;
    public static final double maxLoadedAccelRads = 2.228;

    // PID
    public static final double kP = 200;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double PIDToleranceDegrees = 1.0;

    // FeedForward
    public static final double kS = 0.0;

    public static final double kGEmpty = 0.82;
    public static final double kVEmpty = 12.0 / maxEmptyVelocityRads;

    // public static final double kAEmpty = 12.0 / maxEmptyAccelRads;
    public static final double kAEmpty = 0.6;

    public static final double kGLoaded = 1.02;
    public static final double kVLoaded = 12.0 / maxLoadedVelocityRads;
    // public static final double kALoaded = 12.0 / maxLoadedAccelRads;
    public static final double kALoaded = 0.4;


    // Specs
    public static final double gearing = 120.0;

    public static final double shoulderLength = Units.inchesToMeters(21.0);

    public static final double shoulderMassKg = 2.0;
    public static final double gripperMassKg = 0.75;
    public static final double coneWeightKg = Units.lbsToKilograms(23.0/16.0); // 1lb 7oz

    public static final double totalEmptyArmMassKg = shoulderMassKg + gripperMassKg;
    public static final double totalLoadedArmMassKg = shoulderMassKg + gripperMassKg + coneWeightKg;

    // States
    public enum ArmPositionStates {
        STOW(0.0),
        INTAKE(30.0),
        EJECT(15.0),
        SCORE_LOW(80.0),
        SCORE_HIGH(110.0);

        public final double position_degs;

        ArmPositionStates(double position_degs) {
            this.position_degs = position_degs;
        }
    }

    public enum GripperStates {
        OPEN(false),
        CLOSED(true);

        public final boolean isClosed;

        GripperStates(boolean isClosed){
            this.isClosed = isClosed;
        }
    }

    public enum CargoStates {
        EMPTY(false),
        LOADED(true);

        public final boolean isHoldingCone;

        CargoStates(boolean isHoldingCone){
            this.isHoldingCone = isHoldingCone;
        }
    }
}
