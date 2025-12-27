package frc.robot.subsystems.drive;

public class DriveConstants {

    // PID
    public static final double kP = 0.3;
    public static final double kI = 0.0;
    public static final double kD = 0;

    // FF
    public static final double kS = 0.3;
    public static final double kV = 1.0 / 917.0; // revrobotics.com says Motor Kv: 917 Kv = 0.00109 V per RPM

    public static final double PIDTolerance = 0.5;

}
