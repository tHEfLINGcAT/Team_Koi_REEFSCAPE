package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotHandSubsystem extends SubsystemBase {
    private SparkMax m_SparkMax;
    public RobotHandSubsystem(){
        m_SparkMax=new SparkMax(17, MotorType.kBrushless);
        SparkFlexConfig config=new SparkFlexConfig();
        m_SparkMax.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
        //sets the motor to speed to 1 to take the object
        public void grabObject(){
             //we will check the speed in real time bcz idk what the speed the vortex should be
            m_SparkMax.set(1);
        }
        //sets the motor to -1 bcz I need the other direction and ejects the object
        public void releaseObject(){
        //we will check the speed in real time bcz idk what the speed the vortex should be
            m_SparkMax.set(-1);
        }
}
