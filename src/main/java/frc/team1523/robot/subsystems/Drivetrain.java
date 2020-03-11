package frc.team1523.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1523.robot.Constants;

public class Drivetrain extends SubsystemBase {
    private final WPI_TalonFX leftFront = new WPI_TalonFX(2);
    private final WPI_TalonFX rightRear = new WPI_TalonFX(3);
    private final WPI_TalonFX leftRear = new WPI_TalonFX(4);
    private final WPI_TalonFX rightFront = new WPI_TalonFX(5);
    private final AHRS navx = new AHRS();

    private final SpeedController leftMotors = new SpeedControllerGroup(leftFront, leftRear);
    private final SpeedController rightMotors = new SpeedControllerGroup(rightFront, rightRear);
    private final DifferentialDrive robotDrive = new DifferentialDrive(leftMotors, rightMotors);

    private final PIDController leftPIDController = new PIDController(1, 0, 0);
    private final PIDController rightPIDController = new PIDController(1, 0, 0);

    private final DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(navx.getAngle()));

    private final DifferentialDriveKinematics kinematics
            = new DifferentialDriveKinematics(Constants.DriveConstants.kTrackWidth);
    // practice bot
//    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.175, 2.43, 0.25);
    // comp bot
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(.138, 2.45, .265);

    private int x = 0;

    public Drivetrain() {
        zeroSensors();

        leftFront.configFactoryDefault();
        rightRear.configFactoryDefault();
        leftRear.configFactoryDefault();
        rightFront.configFactoryDefault();

        leftFront.setNeutralMode(NeutralMode.Brake);
        rightRear.setNeutralMode(NeutralMode.Brake);
        leftRear.setNeutralMode(NeutralMode.Brake);
        rightFront.setNeutralMode(NeutralMode.Brake);

        // Don't panic when raw voltages are being set by the pid loops
        robotDrive.setSafetyEnabled(false);
    }

    public void alarm() {
        leftRear.set(ControlMode.MusicTone, x + 450);
        leftFront.set(ControlMode.MusicTone, x + 450);
        rightRear.set(ControlMode.MusicTone, x + 450);
        rightFront.set(ControlMode.MusicTone, x + 450);
        x = (x + 8) % 150;
    }

    public double getAngle() {
        return navx.getAngle();
    }

    public void zeroSensors() {
        driveOdometry.resetPosition(new Pose2d(0.0, 0.0, new Rotation2d()),
                Rotation2d.fromDegrees(navx.getYaw()));
        navx.zeroYaw();
        leftFront.setSelectedSensorPosition(0);
        rightFront.setSelectedSensorPosition(0);
        leftRear.setSelectedSensorPosition(0);
        rightRear.setSelectedSensorPosition(0);
    }

    public void boringDrive(double xSpeed, double rotation) {
        robotDrive.arcadeDrive(xSpeed, rotation);
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @param xSpeed Linear velocity in m/s.
     * @param rot    Angular velocity in rad/s.
     */
    public void fancyDrive(double rot, double xSpeed) {
        var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        setSpeeds(wheelSpeeds);
    }

    public double getLeftDistance() {
        return (leftFront.getSelectedSensorPosition() * Constants.DriveConstants.kDistancePerTick
                + leftRear.getSelectedSensorPosition() * Constants.DriveConstants.kDistancePerTick
        ) / 2.0;
    }

    public double getRightDistance() {
        return (rightFront.getSelectedSensorPosition() * Constants.DriveConstants.kDistancePerTick
                + rightRear.getSelectedSensorPosition() * Constants.DriveConstants.kDistancePerTick
        ) / 2.0;
    }

    public double getAverageDistance() {
        return (getLeftDistance() + getRightDistance()) / 2.0;
    }

    @Override
    public void periodic() {
        driveOdometry.update(Rotation2d.fromDegrees(navx.getYaw()),
                getLeftDistance(),
                getRightDistance());
    }

    /**
     * Sets the desired wheel speeds.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

        final double leftOutput = leftPIDController.calculate(
                leftFront.getSelectedSensorVelocity() * Constants.DriveConstants.kDistancePerTick,
                speeds.leftMetersPerSecond);
        final double rightOutput = rightPIDController.calculate(
                rightFront.getSelectedSensorVelocity() * Constants.DriveConstants.kDistancePerTick,
                speeds.rightMetersPerSecond);
        leftMotors.setVoltage(leftOutput + leftFeedforward);
        rightMotors.setVoltage(rightOutput + rightFeedforward);
    }
}