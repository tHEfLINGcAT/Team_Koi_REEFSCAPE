package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HandRotaionSubSystem extends SubsystemBase {
    private SparkMax m_handRo;
    private boolean finished;
    SparkMaxConfig config = new SparkMaxConfig();
    public HandRotaionSubSystem(){
        m_handRo=new SparkMax(Constants.HandRotaionConstants.CAN_HAND_DEGREE_ID, MotorType.kBrushless);
        config.idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(360.0 / 1024.0)
            .velocityConversionFactor(1024);
        config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(Constants.HandRotaionConstants.HAND_DGREE_SPARKMAX_Kp, Constants.HandRotaionConstants.HAND_DGREE_SPARKMAX_Ki, Constants.HandRotaionConstants.HAND_DGREE_SPARKMAX_Kd);
        m_handRo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_handRo.getEncoder().setPosition(0);
        finished=false;
    }
    public void turnArm(int speed){
        if (speed>0) {
            while(m_handRo.getEncoder().getPosition()*360!=90){
                m_handRo.set(speed);
            }
        }
        if (speed<0) {
            while(m_handRo.getEncoder().getPosition()*360!=0){
                m_handRo.set(-speed);
            }
        }
        finished=true;
    }
    public boolean getFinished(){
        return finished;
    }
}
;