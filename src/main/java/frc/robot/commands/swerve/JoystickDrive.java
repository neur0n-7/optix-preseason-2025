
package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class JoystickDrive extends Command {
    private final SwerveSubsystem swerve;
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    private final DoubleSupplier rotSpeedSupplier;
    private final BooleanSupplier fieldRelativeSupplier;

    public JoystickDrive(
            SwerveSubsystem swerve,
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier,
            DoubleSupplier rotSpeedSupplier,
            BooleanSupplier fieldRelativeSupplier
    ) {
        this.swerve = swerve;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotSpeedSupplier = rotSpeedSupplier;
        this.fieldRelativeSupplier = fieldRelativeSupplier;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.drive(
            xSpeedSupplier.getAsDouble(),
            ySpeedSupplier.getAsDouble(),
            rotSpeedSupplier.getAsDouble(),
            fieldRelativeSupplier.getAsBoolean()
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0, false);
    }
}
