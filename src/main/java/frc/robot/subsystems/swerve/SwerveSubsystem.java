package frc.robot.subsystems.swerve;

import java.lang.constant.Constable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;


public class SwerveSubsystem extends SubsystemBase{

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        SwerveConstants.kFrontLeftLocation,
        SwerveConstants.kFrontRightLocation,
        SwerveConstants.kBackLeftLocation,
        SwerveConstants.kBackRightLocation
    );

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        kinematics,
        new Rotation2d(0.0),
        getModulePositions(),
        new Pose2d()
    );

    public SwerveSubsystem(boolean isReal) {

        if (isReal){
            frontLeft = new SwerveModuleReal(
                OperatorConstants.kFrontLeftDriveMotorPort,
                OperatorConstants.kFrontLeftTurningMotorPort,
                SwerveConstants.kFrontLeftDriveEncoderReversed,
                SwerveConstants.kFrontLeftTurningEncoderReversed,
                SwerveConstants.kFrontLeftDriveAbsoluteEncoderPort,
                SwerveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
                SwerveConstants.kFrontLeftDriveAbsoluteEncoderReversed
            );

            frontRight = new SwerveModuleReal(
                OperatorConstants.kFrontRightDriveMotorPort,
                OperatorConstants.kFrontRightTurningMotorPort,
                SwerveConstants.kFrontRightDriveEncoderReversed,
                SwerveConstants.kFrontRightTurningEncoderReversed,
                SwerveConstants.kFrontRightDriveAbsoluteEncoderPort,
                SwerveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
                SwerveConstants.kFrontRightDriveAbsoluteEncoderReversed
            );

            backLeft = new SwerveModuleReal(
                OperatorConstants.kBackLeftDriveMotorPort,
                OperatorConstants.kBackLeftTurningMotorPort,
                SwerveConstants.kBackLeftDriveEncoderReversed,
                SwerveConstants.kBackLeftTurningEncoderReversed,
                SwerveConstants.kBackLeftDriveAbsoluteEncoderPort,
                SwerveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
                SwerveConstants.kBackLeftDriveAbsoluteEncoderReversed
            );

            backRight = new SwerveModuleReal(
                OperatorConstants.kBackRightDriveMotorPort,
                OperatorConstants.kBackRightTurningMotorPort,
                SwerveConstants.kBackRightDriveEncoderReversed,
                SwerveConstants.kBackRightTurningEncoderReversed,
                SwerveConstants.kBackRightDriveAbsoluteEncoderPort,
                SwerveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
                SwerveConstants.kBackRightDriveAbsoluteEncoderReversed
            );
        } else {
            frontLeft = new SwerveModuleSim();
            frontRight = new SwerveModuleSim();
            backLeft = new SwerveModuleSim();
            backRight = new SwerveModuleSim();
        }
    }


    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    // DRIVE /////////////////////////////////////////////////////////////////////////////////////////

    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
            fieldRelative ?
                edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rotSpeed, getHeading())
                :
                new edu.wpi.first.math.kinematics.ChassisSpeeds(xSpeed, ySpeed, rotSpeed)
        );

        // dont exceed max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.kPhysicalMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }
    
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public Rotation2d getHeading() {
        return new Rotation2d(0); // TODO: replace with gyro reading
    }

    // Pose = position + orientation
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    // because odometry is dumb smh
    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeft.getDrivePosition(),  new Rotation2d(frontLeft.getTurningPosition())),
            new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition())),
            new SwerveModulePosition(backLeft.getDrivePosition(),   new Rotation2d(backLeft.getTurningPosition())),
            new SwerveModulePosition(backRight.getDrivePosition(),  new Rotation2d(backRight.getTurningPosition()))
        };
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getHeading(), getModulePositions(), pose);
    }

    private void logSwerveModule(String name, SwerveModule module) {
        SmartDashboard.putNumber(name + " Drive Pos", module.getDrivePosition());
        SmartDashboard.putNumber(name + " Drive Vel", module.getDriveVelocity());
        SmartDashboard.putNumber(name + " Turn Pos", module.getTurningPosition());
        SmartDashboard.putNumber(name + " Turn Vel", module.getTurningVelocity());
        SmartDashboard.putNumber(name + " Absolute Angle", module.getAbsoluteEncoderRad());
    }

    @Override
    public void periodic(){
        odometry.update(getHeading(), getModulePositions());

        SmartDashboard.putNumber("Robot X", getPose().getX());
        SmartDashboard.putNumber("Robot Y", getPose().getY());
        SmartDashboard.putNumber("Robot Heading", getPose().getRotation().getDegrees());

        logSwerveModule("Front Left", frontLeft);
        logSwerveModule("Front Right", frontRight);
        logSwerveModule("Back Left", backLeft);
        logSwerveModule("Back Right", backRight);
    }

}
