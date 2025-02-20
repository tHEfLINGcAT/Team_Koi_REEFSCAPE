package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveDrive drive;
    private double maximumSpeed = 10.8; // in kilometers per hour

    public double getMaximumSpeed() {
        return maximumSpeed;
    }

    public void setMaximumSpeed(double maximumSpeed) {
        this.maximumSpeed = maximumSpeed;
    }

    public Pose2d getPose() {
        return drive.getPose();
    }

    public void resetOdometry(Pose2d initial) {
        drive.resetOdometry(initial);
    }

    public ChassisSpeeds getRobotVelocity() {
        return drive.getRobotVelocity();
    }

    public void setChassisSpeeds(ChassisSpeeds cs) {
        drive.setChassisSpeeds(cs);
    }

    public SwerveSubsystem(File directory) {
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(
                Constants.GearRatio.SwerveModule.STEERING_GEAR_RATIO,
                Constants.MotorAttributes.Neo.COUNTS_PER_REVOLUTION_SparkMax);

        double wheelDiameter = Constants.Convert.INCHES_TO_METERS * 4;

        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(
                wheelDiameter,
                Constants.GearRatio.SwerveModule.DRIVE_GEAR_RATIO,
                Constants.MotorAttributes.Vortex.COUNTS_PER_REVOLUTION_SparkFlex);

        try {
            drive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
        } catch (Exception e) {
            throw new RuntimeException(
                    "Error with directory. Please make sure the given File object is a directory and not a file");
        }

        drive.setHeadingCorrection(false);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        drive.drive(translation, rotation, fieldRelative, false);
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        drive.driveFieldOriented(velocity);
    }

    public void drive(ChassisSpeeds velocity) {
        drive.drive(velocity);
    }

    public void periodic() {}

    public void simulationPeriodic() {}

    public SwerveDriveKinematics getKinematics() {
        return drive.kinematics;
    }

    public void postTrajectory(Trajectory trajectory) {
        drive.postTrajectory(trajectory);
    }

    public void zeroGyro() {
        drive.zeroGyro();
    }

    public void setMotorBrake(boolean brake) {
        drive.setMotorIdleMode(brake);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return drive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians(), maximumSpeed);
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return drive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians(), maximumSpeed);
    }

    public ChassisSpeeds getFieldVelocity() {
        return drive.getFieldVelocity();
    }

    public SwerveController getSwerveController() {
        return drive.swerveController;
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return drive.swerveDriveConfiguration;
    }

    public void lock() {
        drive.lockPose();
    }

    public Rotation2d getPitch() {
        return drive.getPitch();
    }

}
