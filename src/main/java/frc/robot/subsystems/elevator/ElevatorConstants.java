package frc.robot.subsystems.elevator;

public class ElevatorConstants {

    // PID
    public static double kP = 12;
    public static double kI = 0;
    public static double kD = 0;

    // FeedForward

    public static double kG = 0.32; // gravity FF
    public static double kS = 0.16; // friction FF
    public static double kV = 7.77; // voltage FF (maintain velocity)
    public static double kA = 0.27; // acceleration? (predict acceleration effort)

    // Other stuff

    public static double maxVelocity = 1.415;
    public static double maxAccel = 4.1;

    public enum ElevatorStates {
        LOWEST(0),
        HIGHEST(6);

        public final int position;

        ElevatorStates(int position) {
            this.position = position;
        }
    }

}
