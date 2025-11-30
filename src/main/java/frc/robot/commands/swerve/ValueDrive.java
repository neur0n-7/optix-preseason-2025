
/*
 * Like JoystickDrive.java except we feed this xSpeed, ySpeed, and rotSpeed values
 * instead of using suppliers 
 */

package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ValueDrive extends Command {
    private final SwerveSubsystem swerve;
    private final double xSpeed;
    private final double ySpeed;
    private final double rotSpeed;
    private final BooleanSupplier fieldRelativeSupplier;

    public ValueDrive(
            SwerveSubsystem swerve,
            double xSpeed,
            double ySpeed,
            double rotSpeed,
            BooleanSupplier fieldRelativeSupplier
    ) {
        this.swerve = swerve;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotSpeed = rotSpeed;
        this.fieldRelativeSupplier = fieldRelativeSupplier;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.drive(
            xSpeed,
            ySpeed,
            rotSpeed,
            fieldRelativeSupplier.getAsBoolean()
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0, false);
    }
}
