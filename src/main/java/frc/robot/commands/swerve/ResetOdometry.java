package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ResetOdometry extends Command{
    
    private final SwerveSubsystem swerve;
    private final Pose2d pose;

    public ResetOdometry(SwerveSubsystem swerve, Pose2d pose){
        this.swerve = swerve;
        this.pose = pose;
        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        swerve.resetOdometry(pose);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
