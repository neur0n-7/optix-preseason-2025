package frc.robot.subsystems.swerve;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModuleReal extends SubsystemBase implements SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPIDController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModuleReal(int driveMotorID, int turningMotorID, 
        boolean driveMotorReversed, boolean turningMotorReversed,
        int absoluteEncoderID, double absoluteEncoderOffset,
        boolean absoluteEncoderReversed){

            absoluteEncoder = new AnalogInput(absoluteEncoderID);
            this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
            this.absoluteEncoderReversed = absoluteEncoderReversed;
            
            driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
            turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);

            // These functions are deprecated but I'm (temporarily) using them
            driveMotor.setInverted(driveMotorReversed);
            turningMotor.setInverted(turningMotorReversed);

            driveEncoder = driveMotor.getEncoder();
            turningEncoder = driveMotor.getEncoder();

            turningPIDController = new PIDController(0.1, 0.0, 0.0);
            turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        
            resetEncoders();
        }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.set(volts / 12.0); // convert volts to percent
    }

    @Override
    public void setTurnVoltage(double volts) {
        turningMotor.set(volts / 12.0);
    }

    @Override
    public double getDrivePosition(){ return driveEncoder.getPosition(); }

    @Override
    public double getTurningPosition(){ return turningEncoder.getPosition(); }

    @Override
    public double getDriveVelocity(){ return driveEncoder.getVelocity(); }

    @Override
    public double getTurningVelocity(){ return turningEncoder.getVelocity(); }

    @Override
    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    @Override
    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());

    }

    @Override
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    @Override
    public void setDesiredState(SwerveModuleState state){

        if (Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / SwerveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    @Override
    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
