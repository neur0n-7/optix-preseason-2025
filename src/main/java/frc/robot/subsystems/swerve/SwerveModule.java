package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {

    public void setDriveVoltage(double volts);

    public void setTurnVoltage(double volts);

    public double getDrivePosition();

    public double getTurningPosition();

    public double getDriveVelocity();

    public double getTurningVelocity();

    public double getAbsoluteEncoderRad();

    public void resetEncoders();

    public SwerveModuleState getState();

    public void setDesiredState(SwerveModuleState state);

    public void stop();
}
