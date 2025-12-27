package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModuleSim extends SubsystemBase implements SwerveModuleIO {

    // properties
    private double drivePos = 0.0;
    private double driveVel = 0.0;

    private double turnPos = 0.0;   // radians
    private double turnVel = 0.0;   // rad/s

    // Inputs
    private double driveVolts = 0.0;
    private double turnVolts = 0.0;

    private final PIDController turningPIDController;

    // sim constants
    private static final double DRIVE_kV = 3.0;     // m/s per volt (tune)
    private static final double DRIVE_kA = 0.6;     // m/s^2 per volt
    private static final double TURN_kV  = 4.0;     // rad/s per volt
    private static final double TURN_kA  = 10.0;

    public SwerveModuleSim() {

        turningPIDController = new PIDController(0.1, 0.0, 0.0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void setDriveVoltage(double volts) { driveVolts = volts; }

    @Override
    public void setTurnVoltage(double volts) { turnVolts = volts; }
    

    @Override
    public double getDrivePosition() { return drivePos; }

    @Override
    public double getTurningPosition() { return turnPos; }

    @Override
    public double getDriveVelocity() { return driveVel; }

    @Override
    public double getTurningVelocity() { return turnVel; }

    @Override
    public double getAbsoluteEncoderRad() {
        // turning angle wrapped to 0-2*pi
        return Rotation2d.fromRadians(turnPos).getRadians();
    }

    @Override
    public void resetEncoders() {
        drivePos = 0;
        turnPos = getAbsoluteEncoderRad();
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveVel, new Rotation2d(turnPos));
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, new Rotation2d(turnPos));
        driveVolts = state.speedMetersPerSecond / SwerveConstants.ControlConstants.teleopMaxSpeedMetersPerSecond * 12.0;
        double output = turningPIDController.calculate(turnPos, state.angle.getRadians());
        turnVolts = output * 12.0;
    }

    @Override
    public void stop() {
        driveVolts = 0;
        turnVolts = 0;
    }

    @Override
    public void periodic() {
        double dt = 0.02;

        // drive
        double driveAccel = ((driveVolts * DRIVE_kV) - driveVel) * DRIVE_kA;
        driveVel += driveAccel * dt;
        drivePos += driveVel * dt;        

        // turn
        double turnAccel = ((turnVolts * TURN_kV) - turnVel) * TURN_kA;
        turnVel += turnAccel * dt;
        turnPos += turnVel * dt;
        
        // wrap angle
        turnPos = Math.atan2(Math.sin(turnPos), Math.cos(turnPos));        
    }
}
