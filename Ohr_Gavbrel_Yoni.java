package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private static final int MOTOR_ID = 1; 
    private static final double MAX_ANGLE = 100.0; 
    private static final double MIN_ANGLE = 0.0; 
    private static final double SPEED_LIMIT = 0.5; 

    private final CANSparkMax armMotor;
    private final SparkMaxAbsoluteEncoder encoder;

    public ArmSubsystem() {
        armMotor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
        encoder = armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        armMotor.setIdleMode(IdleMode.kBrake); 
        encoder.setPositionConversionFactor(360.0 / 100.0); 
    }

    public void moveArm(double speed) {
        speed = Math.copySign(Math.min(Math.abs(speed), SPEED_LIMIT), speed); 
        double angle = getArmAngle();
        
        if ((speed > 0 && angle < MAX_ANGLE) || (speed < 0 && angle > MIN_ANGLE)) {
            armMotor.set(speed);
        } else {
            armMotor.set(0);
        }
    }

    public double getArmAngle() {
        double angle = encoder.getPosition() * 360; 
        SmartDashboard.putNumber("Arm Angle", angle); 
        return angle;
    }

    public void stopArm() {
        armMotor.set(0);
    }
}
