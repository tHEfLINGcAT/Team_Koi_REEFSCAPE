package frc.robot.commands.DeliveryCatchCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DeliveryCatchSubsystem.DeliveryCatchSubsystem;

public class ReleaseGamePieceCommand extends CommandBase {
    private final DeliveryCatchSubsystem deliveryCatchSubsystem;

    public ReleaseGamePieceCommand(DeliveryCatchSubsystem subsystem) {
        deliveryCatchSubsystem = subsystem;
        addRequirements(deliveryCatchSubsystem);
    }

    @Override
    public void execute() {
        deliveryCatchSubsystem.release(); //calls release
    }

    @Override
    public void end(boolean interrupted) {
        deliveryCatchSubsystem.stop(); //stops the motor when released
    }
}
