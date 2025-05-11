package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RotationPresetSubsystem;

public class PresetRotationCommand extends Command {
    private final RotationPresetSubsystem rotationSubsystem;
    private final boolean PositiveIncrement;

    public PresetRotationCommand(RotationPresetSubsystem rotationSubsystem, boolean PositiveIncrement) {
        this.rotationSubsystem = rotationSubsystem;
        this.PositiveIncrement = PositiveIncrement;
        addRequirements(rotationSubsystem);
    }

    @Override
    public void execute() {
        rotationSubsystem.rotateToCurrentHeading();  // This should be checking if you're at the target heading
    }

    @Override
    public boolean isFinished() {
        return rotationSubsystem.rotateToCurrentHeading();  // This should return true when done rotating
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            return;  // No action needed if interrupted
        }
        if (PositiveIncrement) rotationSubsystem.next();
        else rotationSubsystem.previous();
    }
}
