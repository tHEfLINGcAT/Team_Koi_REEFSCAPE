package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HandRotaionSubSystem extends SubsystemBase {
    private SparkMax m_handRo;
    private boolean finished, isInverted, noSuicide;
    private final PIDController pidController;
    double offset;
    DutyCycleEncoder encoder;
    SparkMaxConfig config = new SparkMaxConfig();

    public HandRotaionSubSystem() {
        encoder = new DutyCycleEncoder(Constants.HandRotaionConstants.HAND_DGREE_ENCODER_PORT, 360,
                Constants.HandRotaionConstants.HAND_DGREE_ENCODER_OFFSET);
        m_handRo = new SparkMax(Constants.HandRotaionConstants.CAN_HAND_DEGREE_ID, MotorType.kBrushless);
        config.idleMode(IdleMode.kBrake);
        pidController = new PIDController(Constants.HandRotaionConstants.HAND_DGREE_SPARKMAX_Kp,
                Constants.HandRotaionConstants.HAND_DGREE_SPARKMAX_Ki,
                Constants.HandRotaionConstants.HAND_DGREE_SPARKMAX_Kd);
        // SmartDashboard.putNumber("P hand Gain",Constants.ArmConstants.Kp);
        // SmartDashboard.putNumber("D hand Gain", Constants.ArmConstants.Kd);
        m_handRo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void turnArm(Double angle) {
        if (getPosition() <= 273 && getPosition() >= 179) {
            m_handRo.setVoltage(pidController.calculate(getPosition(), angle));
        } else {
            finished = true;
            m_handRo.setVoltage(0);
        }
    }

    public boolean isInverted() {
        return isInverted;
    }

    public void endMotor() {
        m_handRo.set(0);
    }

    public boolean getFinished() {
        return finished;
    }

    public double getPosition() {
        return encoder.get();
    }

    public void periodic() {
        SmartDashboard.putNumber("curret rotaion postion", (int) getPosition());
        SmartDashboard.putBoolean("is connnected rotate", encoder.isConnected());
        System.out.println(isInverted);
    }
};
