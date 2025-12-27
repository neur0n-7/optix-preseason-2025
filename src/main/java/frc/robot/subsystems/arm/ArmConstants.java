package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {

    // Constraints
    public static final double maxEmptyVelocity = 0.0;
    public static final double maxEmptyAccel = 0.0;
    
    public static final double maxLoadedVelocity = 0.0;
    public static final double maxLoadedAccel = 0.0;

    // PID
    public static final double kP = 2;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double PIDTolerance = 1.0;

    // FeedForward
    public static final double kS = 0.0;

    public static final double kGEmpty = 0.0;
    public static final double kVEmpty = 12.0 / maxEmptyVelocity;
    public static final double kAEmpty = 12.0 / maxEmptyAccel;

    public static final double kGLoaded = 0.0;
    public static final double kVLoaded = 12.0 / maxLoadedVelocity;
    public static final double kALoaded = 12.0 / maxLoadedAccel;

    // Specs
    public static final double gearing = 5.0;

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
        SCORE_LOW(60.0),
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
        EMPTY(false, totalEmptyArmMassKg),
        LOADED(true, totalLoadedArmMassKg);

        public final boolean isHoldingCone;
        public final double armMass;

        CargoStates(boolean isHoldingCone, double armMass){
            this.isHoldingCone = isHoldingCone;
            this.armMass = armMass;
        }
    }
}
