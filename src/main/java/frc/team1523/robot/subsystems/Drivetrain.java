package frc.team1523.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1523.robot.Constants;

public class Drivetrain extends SubsystemBase {
    private final TalonFX leftFront = new TalonFX(2);
    private final TalonFX rightRear = new TalonFX(3);
    private final TalonFX leftRear = new TalonFX(4);
    private final TalonFX rightFront = new TalonFX(5);
    private final AHRS navx = new AHRS(NavXComType.kMXP_SPI);

    private final DifferentialDrive robotDrive = new DifferentialDrive(leftFront::set, rightFront::set);

    private final PIDController leftPIDController = new PIDController(1, 0, 0);
    private final PIDController rightPIDController = new PIDController(1, 0, 0);

    private final DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(navx.getAngle()), 0, 0);

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
            Constants.DriveConstants.kTrackWidth);
    // practice bot
    // private final SimpleMotorFeedforward feedforward = new
    // SimpleMotorFeedforward(0.175, 2.43, 0.25);
    // comp bot
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(.138, 2.45, .265);

    private int x = 0;

    public Drivetrain() {
        zeroSensors();

        var leftConfigs = new TalonFXConfiguration();
        leftConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        var rightConfigs = new TalonFXConfiguration();
        rightConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        

        leftFront.getConfigurator().apply(leftConfigs);
        leftRear.getConfigurator().apply(leftConfigs);
        rightFront.getConfigurator().apply(rightConfigs);
        rightRear.getConfigurator().apply(rightConfigs);

        leftRear.setControl(new Follower(leftFront.getDeviceID(), false));
        rightRear.setControl(new Follower(rightFront.getDeviceID(), false));

        // Don't panic when raw voltages are being set by the pid loops
        robotDrive.setSafetyEnabled(false);
    }

    public void alarm() {
        leftRear.setControl(new MusicTone(x + 450));
        leftFront.setControl(new MusicTone(x + 450));
        rightRear.setControl(new MusicTone(x + 450));
        rightFront.setControl(new MusicTone(x + 450));
        x = (x + 8) % 150;
    }

    public double getAngle() {
        return navx.getAngle();
    }

    public void zeroSensors() {
        // driveOdometry.resetPosition(new Pose2d(0.0, 0.0, new Rotation2d()),
        // Rotation2d.fromDegrees(navx.getYaw()));
        navx.zeroYaw();
        leftFront.setPosition(0);
        rightFront.setPosition(0);
        leftRear.setPosition(0);
        rightRear.setPosition(0);
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
        return (leftFront.getPosition().getValueAsDouble() * Constants.DriveConstants.kDistancePerTick
                + leftRear.getPosition().getValueAsDouble() * Constants.DriveConstants.kDistancePerTick) / 2.0;
    }

    public double getRightDistance() {
        return -(rightFront.getPosition().getValueAsDouble() * Constants.DriveConstants.kDistancePerTick
                + rightRear.getPosition().getValueAsDouble() * Constants.DriveConstants.kDistancePerTick) / 2.0;
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
                leftFront.getVelocity().getValueAsDouble() * Constants.DriveConstants.kDistancePerTick,
                speeds.leftMetersPerSecond);
        final double rightOutput = rightPIDController.calculate(
                rightFront.getVelocity().getValueAsDouble() * Constants.DriveConstants.kDistancePerTick,
                speeds.rightMetersPerSecond);
        
        leftFront.setVoltage(leftOutput + leftFeedforward);
        rightFront.setVoltage(-(rightOutput + rightFeedforward));
    }
}