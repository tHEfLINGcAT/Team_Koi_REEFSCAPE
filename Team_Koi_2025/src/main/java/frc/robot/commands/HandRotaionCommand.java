package frc.robot.commands;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandRotaionSubSystem;

public class HandRotaionCommand extends Command {
    public final HandRotaionSubSystem handRoSubsystem;
    private double degree;
    public HandRotaionCommand(HandRotaionSubSystem handRo,double angle){
        handRoSubsystem=handRo;
        degree=angle;
        addRequirements(handRo);
    }

    public void execute(){
        handRoSubsystem.turnArm(degree);
      //  System.out.println("COMMAND ROTATE");
  }
    @Override
    public boolean isFinished() {
        //when the motor got to the right angle
        return handRoSubsystem.getFinished();
    }
}
