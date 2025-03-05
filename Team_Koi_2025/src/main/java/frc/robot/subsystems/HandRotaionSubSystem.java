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
    public HandRotaionSubSystem() {
        // Initialize the motor as a brushless Spark Max
        m_handRo = new SparkMax(Constants.HandRotaionConstants.CAN_HAND_DEGREE_ID, MotorType.kBrushless);
        
        // The arm should now stop when idle and not just fall
        config.idleMode(IdleMode.kBrake);

        // encoder configuration
        config.encoder
            .positionConversionFactor(360.0 / 1024.0)
            .velocityConversionFactor(1024);

        //PID and sensors
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.HandRotaionConstants.HAND_DGREE_SPARKMAX_Kp, Constants.HandRotaionConstants.HAND_DGREE_SPARKMAX_Ki, Constants.HandRotaionConstants.HAND_DGREE_SPARKMAX_Kd);
            m_handRo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_handRo.getEncoder().setPosition(0);

        //set finished to false
        finished = false;
    }

    /**
     * Rotates the arm to a specified target position using PID control.
     * @param targetDegrees The target position in degrees (0 to 90).
     */
    public void turnArm(double targetDegrees) {
        // Ensure target is within safe limits (0 to 90 degrees)
        targetDegrees = Math.max(0, Math.min(90, targetDegrees));
        
        // Set power untill we finally go to the wanted degree
        armTurner(targetDegrees,  m_handRo.getEncoder().getPosition() < targetDegrees);
        finished = false;
    }

    private void armTurner(double targetDegrees, boolean positive) {
        // apply force untill we're in range of desired degree
        while(Math.abs((m_handRo.getEncoder().getPosition())-targetDegrees) > 1) {
            m_handRo.set(positive ? Constants.HandRotaionConstants.TURNING_SPEED : -Constants.HandRotaionConstants.TURNING_SPEED);
        }
    }

    /**
     * Checks if the arm has reached the target position.
     * @return true if the arm is within 1 degree of the target, false otherwise.
     */
    public boolean isFinished() {
        return finished;
    }
}