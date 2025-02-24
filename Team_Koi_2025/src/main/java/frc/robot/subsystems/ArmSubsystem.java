package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private final SparkMax armMotor;
    private final SparkMaxConfig config;
    private double armAngle;
    private double offset=Constants.ArmConstants.ENCODER_OFFSET;
    private boolean finished;

    public ArmSubsystem() {
        // Initialize the motor with the specified ID and motor type
        armMotor = new SparkMax(Constants.ArmConstants.MOTOR_ID, SparkMax.MotorType.kBrushless);

        // Create a new configuration object
        config = new SparkMaxConfig();

        // Set the idle mode to brake
        config.idleMode(IdleMode.kBrake);
        config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(Constants.ArmConstants.Kp, Constants.ArmConstants.Ki, Constants.ArmConstants.Kd, 0.0);

        // Apply the configuration to the motor controller
        armMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // Reset the encoder position to zero
        armMotor.getEncoder().setPosition(offset);
        finished=false;
    }

    public void moveArm(double target) {
        // Limit the speed to the defined maximum
        target = Math.copySign(Math.min(Math.abs(target), Constants.ArmConstants.SPEED_LIMIT), target);
        armAngle = getArmAngle();

        // Move the arm only if it's within the allowed angle range
        if ((target > 0 && armAngle < Constants.ArmConstants.MAX_ANGLE) || (target < 0 && armAngle > Constants.ArmConstants.MIN_ANGLE)) {
            armMotor.getClosedLoopController().setReference(target,SparkMax.ControlType.kPosition,ClosedLoopSlot.kSlot0, getFF());
        } else {
            armMotor.set(0);
        }
        finished=true;
    }

    public double getArmAngle() {
        // Calculate the arm angle based on the encoder position
        armAngle = armMotor.getEncoder().getPosition() * 360.0;
        SmartDashboard.putNumber("Arm Angle", armAngle);
        return armAngle;
    }

    public void stopArm() {
        // Stop the arm motor
        armMotor.set(0);
    }
    public boolean getFinished(){
        return finished;
    }
    private double getFF(){
        return Constants.ArmConstants.FF * Math.sin(Units.degreesToRadians(getArmAngle()) ) ;
    }
    @Override
    public void periodic() {
        
        super.periodic();
    }
}
