package frc.robot.subsystems.arm;

public class ArmConstants {

    // pid, ff, and specs are all taken from optix 2025 code (for now)

    // ARM CONTROL ////////////////////////////////////////////////////////
    // PID
    public static double kP = 2.22;
    public static double kI = 0;
    public static double kD = 0;

    // FeedForward
    // stolen from Team-3749-2025 for now :))
    public static double kG = 0.39; // gravity FF
    public static double kS = 0.16; // friction FF
    public static double kV = 0.8; // voltage FF (maintain velocity)
    public static double kA = 0; // acceleration? (predict acceleration effort)

    // Other stuff

    // ARM STATES ////////////////////////////////////////////////////////
    public enum ArmStates {
        DOWN(0),
        UP(180);

        public final int position_degs;

        ArmStates(int position_degs) {
            this.position_degs = position_degs;
        }
    }

}
