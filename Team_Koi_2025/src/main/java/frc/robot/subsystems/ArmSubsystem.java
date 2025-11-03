package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private final SparkMax armMotorLeft, armMotorRight;
    private final SparkClosedLoopController pidLeft, pidRight;
    private final ArmFeedforward feedforward;

    private double armAngleDeg;
    private boolean finished;

    private final DutyCycleEncoder abEncoderLeft, abEncoderRight;

    private static final double ANGLE_TOLERANCE_DEG = 2.0;

    public ArmSubsystem() {
        armMotorLeft = new SparkMax(Constants.ArmConstants.MOTOR_ID_LEFT, SparkMax.MotorType.kBrushless);
        armMotorRight = new SparkMax(Constants.ArmConstants.MOTOR_ID_RIGHT, SparkMax.MotorType.kBrushless);

        abEncoderLeft = new DutyCycleEncoder(Constants.ArmConstants.ENCODER_PORT_LEFT);
        abEncoderRight = new DutyCycleEncoder(Constants.ArmConstants.ENCODER_PORT_RIGHT);

        // sync relative encoders to absolute (convert 0–1 to degrees)
        armMotorLeft.getEncoder().setPosition(abEncoderLeft.get() * 360.0);
        armMotorRight.getEncoder().setPosition(abEncoderRight.get() * 360.0);

        // Config Spark PID + Kv
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);

        config.closedLoop
            .p(Constants.ArmConstants.Kp)
            .i(Constants.ArmConstants.Ki)
            .d(Constants.ArmConstants.Kd)
            .velocityFF(1.0 / Constants.ArmConstants.Kv);

        armMotorLeft.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        armMotorRight.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        pidLeft = armMotorLeft.getClosedLoopController();
        pidRight = armMotorRight.getClosedLoopController();

        feedforward = new ArmFeedforward(
            Constants.ArmConstants.Ks,
            Constants.ArmConstants.Kg,
            Constants.ArmConstants.Kv,
            Constants.ArmConstants.Ka
        );

        finished = false;
    }

    public void moveArm(double targetAngleDeg, double targetVelocityDegPerSec) {
        armAngleDeg = getArmAngleDeg();

        if (targetAngleDeg >= Constants.ArmConstants.MIN_ANGLE &&
            targetAngleDeg <= Constants.ArmConstants.MAX_ANGLE) {

            double arbFFVolts = feedforward.calculate(
                Math.toRadians(armAngleDeg), 
                Math.toRadians(targetVelocityDegPerSec), 
                0.0
            );

            pidLeft.setReference(targetAngleDeg, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFFVolts);
            pidRight.setReference(targetAngleDeg, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFFVolts);

            finished = Math.abs(targetAngleDeg - armAngleDeg) < ANGLE_TOLERANCE_DEG;
        } else {
            stopArm();
            finished = true;
        }
    }

    public double getArmAngleDeg() {
        return armMotorLeft.getEncoder().getPosition();
    }

    public void stopArm() {
        armMotorLeft.set(0);
        armMotorRight.set(0);
    }

    public boolean getFinished() {
        if (finished) stopArm();
        return finished;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm angle position (deg)", getArmAngleDeg());
        SmartDashboard.putBoolean("Left encoder connected", abEncoderLeft.isConnected());
        SmartDashboard.putBoolean("Right encoder connected", abEncoderRight.isConnected());
    }
}
