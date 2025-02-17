package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants;
import frc.robot.commands.ElevatorCommand;

public class ElevatorSubsystem extends SubsystemBase{
    private final SparkMax motor;
    private final SparkMaxConfig config = new SparkMaxConfig();

    public ElevatorSubsystem(){
       this.motor = new SparkMax(Constants.ElevatorConstants.ELEVATE_MOTOR_ID, MotorType.kBrushless);
       setDefaultCommand(new ElevatorCommand(this));

       config.idleMode(IdleMode.kBrake);
       config.encoder
            .positionConversionFactor(1000)
            .velocityConversionFactor(1000);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.PIDConstants.kP, Constants.PIDConstants.kI, Constants.PIDConstants.kD);
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTargetPosition(double targetPosition) {
        motor.getClosedLoopController().setReference(targetPosition, ControlType.kPosition);
    }

    public void setTargetVelocity(double targetVelocity) {
        motor.getClosedLoopController().setReference(targetVelocity, ControlType.kVelocity);
    }
}
