package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {

    // ELEVATOR CONTROL ////////////////////////////////////////////////////////
    // PID
    public static double kP = 12;
    public static double kI = 0;
    public static double kD = 0;

    // FeedForward
    // stolen from Team-3749-2025 for now :))
    public static double kG = 0.32; // gravity FF
    public static double kS = 0.16; // friction FF
    public static double kV = 7.77; // voltage FF (maintain velocity)
    public static double kA = 0.27; // acceleration? (predict acceleration effort)

    // Other stuff

    public static double maxVelocity = 1.415;
    public static double maxAccel = 4.1;

    // ELEVATOR SPECS ////////////////////////////////////////////////////////
    // numbers also taken from optix 2025 code, can be changed to use actual specs

    public static final double gearing = 16 * (24.0 / 22.0);
    public static final double elevatorBaseHeight = Units.feetToMeters(3.25); // meters

    // ELEVATOR STATES ////////////////////////////////////////////////////////
    public enum ElevatorStates {
        LOWEST(0),
        MIDDLE(3),
        HIGHEST(6);

        public final int position;

        ElevatorStates(int position) {
            this.position = position;
        }
    }

}
