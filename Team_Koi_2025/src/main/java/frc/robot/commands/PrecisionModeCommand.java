package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotHandSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.RobotHandConstants;;

/*
 * this class turns on and off the precision mode for the driver
 */
public class PrecisionModeCommand extends Command {
    private final SwerveSubsystem swerve;
    public PrecisionModeCommand(SwerveSubsystem swerve){
        this.swerve = swerve;
    }
    
    @Override
    public void execute(){
        swerve.SetPrecisionMode(true);
    }

    @Override
    public void end(boolean interrupted){
        swerve.SetPrecisionMode(false);
    }
}