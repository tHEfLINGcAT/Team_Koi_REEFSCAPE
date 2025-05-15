package frc.robot.commands;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command{
    private final ArmSubsystem m_ArmSubsystem;
    private final double angle;
    private final double velocity;
    public ArmCommand(ArmSubsystem subsystem,double anglle,double velcoiity){
        m_ArmSubsystem=subsystem;
        angle=anglle;
        velocity=velcoiity;
        addRequirements(subsystem);
    }
    public void execute(){
        m_ArmSubsystem.moveArm(angle,velocity);
        System.out.println("MOVE");
    }

   // public void end(boolean interrupted){
    //    m_ArmSubsystem.stopArm();
    //    System.out.println("I ENDED");
   // }

    @Override
    public boolean isFinished() {
        return m_ArmSubsystem.getFinished();
    }
}
