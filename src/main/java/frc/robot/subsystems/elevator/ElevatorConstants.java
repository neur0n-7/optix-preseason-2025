package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {

    // ELEVATOR CONTROL ////////////////////////////////////////////////////////
    // PID
    public static double kP = 5;
    public static double kI = 0;
    public static double kD = 0;

    // FeedForward
    // stolen from Team-3749-2025 for now :))
    public static double kG = 0.32; // gravity FF
    public static double kS = 0.16; // friction FF
    public static double kV = 1.0; // voltage FF (maintain velocity)
    public static double kA = 0.05; // acceleration? idk how this works :(

    // Other stuff

    public static double maxVelocity = 10; // meters/s
    public static double maxAccel = 20; // meters/s^2

    // ELEVATOR SPECS ////////////////////////////////////////////////////////
    // numbers also taken from optix 2025 code, can be changed to use actual specs

    public static final double gearing = 16 * (24.0 / 22.0);
    public static final double elevatorBaseHeight = Units.feetToMeters(3.25); // meters


    public static final double drumDiameter = Units.inchesToMeters(1.5);
    public static final double drumCircumference = Math.PI * drumDiameter;

    public static final double metersPerMotorRotation = drumCircumference / gearing;
    // ELEVATOR STATES ////////////////////////////////////////////////////////
    public enum ElevatorStates {
        LOWEST(Units.feetToMeters(0)),
        MIDDLE(Units.feetToMeters(3)),
        HIGHEST(Units.feetToMeters(6));

        public final double position;

        ElevatorStates(double position) {
            this.position = position;
        }
    }

}
