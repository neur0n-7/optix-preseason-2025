package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;

public class SwerveConstants {

    // Below values are test values

    public static final double maxAngularSpeed = 3.0;
    public static final double maxAngularAcceleration = 3.0;

    public static final Translation2d kFrontLeftLocation  = new Translation2d(+0.3, +0.3);
    public static final Translation2d kFrontRightLocation = new Translation2d(+0.3, -0.3);
    public static final Translation2d kBackLeftLocation   = new Translation2d(-0.3, +0.3);
    public static final Translation2d kBackRightLocation  = new Translation2d(-0.3, -0.3);


    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
    public static final int kBackRightDriveAbsoluteEncoderPort = 3;


    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -4.811;
    
}
