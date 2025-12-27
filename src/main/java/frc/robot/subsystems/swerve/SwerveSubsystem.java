package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveSubsystem extends SubsystemBase{

    private final SwerveModuleIO[] modules = new SwerveModuleIO[4];

    private final SwerveDriveKinematics kinematics = SwerveConstants.DrivetrainConstants.driveKinematics;

    public SwerveSubsystem(boolean isReal) {
        for (int i = 0; i < 4; i++) {
            if (isReal) {
                modules[i] = new SwerveModuleReal(
                        SwerveConstants.MotorConstants.driveMotorIds[i],
                        SwerveConstants.MotorConstants.turnMotorIds[i],
                        SwerveConstants.MotorConstants.driveMotorInverted[i],
                        SwerveConstants.MotorConstants.turningMotorInverted[i],
                        SwerveConstants.MotorConstants.absoluteEncoderIds[i],
                        SwerveConstants.MotorConstants.absoluteEncoderOffsetRad[i],
                        SwerveConstants.MotorConstants.driveAbsoluteEncoderInverted[i]
                );
            } else {
                modules[i] = new SwerveModuleSim();
            }
        }
    }

    // DRIVE /////////////////////////////////////////////////////////////////////////////////////////

    public void drive(double xSpeed, double ySpeed, double rotSpeed) {

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
            new edu.wpi.first.math.kinematics.ChassisSpeeds(xSpeed, ySpeed, rotSpeed)
        );

        // dont exceed max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.ControlConstants.teleopMaxSpeedMetersPerSecond);

        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(states[i]);
        }
    }

    private void logSwerveModule(String name, SwerveModuleIO module) {
        SmartDashboard.putNumber(name + " Drive Pos", module.getDrivePosition());
        SmartDashboard.putNumber(name + " Drive Vel", module.getDriveVelocity());
        SmartDashboard.putNumber(name + " Turn Pos", module.getTurningPosition());
        SmartDashboard.putNumber(name + " Turn Vel", module.getTurningVelocity());
        SmartDashboard.putNumber(name + " Absolute Angle", module.getAbsoluteEncoderRad());
    }

    @Override
    public void periodic(){
        String[] moduleNames = {"FL", "FR", "BL", "BR"};

        for (int i=0; i<4; i++){
            logSwerveModule("Swerve/%s Module".formatted(moduleNames[i]), modules[i]);

        }

        SwerveModuleState[] states = new SwerveModuleState[] {
            modules[0].getState(),
            modules[1].getState(),
            modules[2].getState(),
            modules[3].getState(),
        };

        Logger.recordOutput("Swerve/States", states);

    }

}
