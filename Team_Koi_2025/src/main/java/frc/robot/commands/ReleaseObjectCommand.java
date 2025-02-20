package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotHandSubsystem;

public class ReleaseObjectCommand extends Command {
    private final RobotHandSubsystem m_HandSubsystem;
    public ReleaseObjectCommand(RobotHandSubsystem hand){
        m_HandSubsystem=hand;
        addRequirements(hand);
    }
    
  public void initialize(){
        m_HandSubsystem.releaseObject();
  }
  public boolean isFinished(){
    return true;
  }

}
