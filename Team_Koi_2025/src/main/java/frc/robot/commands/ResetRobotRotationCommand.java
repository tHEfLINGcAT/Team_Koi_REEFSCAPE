package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RotationPresetSubsystem;

public class ResetRobotRotationCommand extends Command {
    private final RotationPresetSubsystem rps;

    public ResetRobotRotationCommand(RotationPresetSubsystem rps) {
        this.rps = rps;
        addRequirements(rps);
    }

    @Override
    public void execute() {
        rps.rotateToHeadingIndex(0); // always rotate to index 0
    }

    @Override
    public boolean isFinished() {
        return rps.rotateToHeadingIndex(0); // returns true when rotation to index 0 is complete
    }
}
