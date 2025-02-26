package frc.team1523.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1523.robot.Constants;

public class Shooter extends SubsystemBase {
    private final SparkMax intakeAgitator = new SparkMax(11, SparkMax.MotorType.kBrushed);
    // private final CANSparkMax shooterFeeder = new CANSparkMax(10,
    // CANSparkMax.MotorType.kBrushless);
    private final SparkMax shooterMotor = new SparkMax(8, SparkMax.MotorType.kBrushless);

    private boolean shooting = false;
    private boolean slow = false;


    public void slowYes() {
        slow = true;
    }

    public void slowNo() {
        slow = false;
    }

    public Shooter() {
        SparkBaseConfig agitatorConfig = new SparkMaxConfig().inverted(false);
        intakeAgitator.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkBaseConfig shooterConfig = new SparkMaxConfig();
        shooterConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(50);

        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {

        if (shooting && !slow) {
            // Only load balls if we are at speed
            if (shooterMotor.getEncoder().getVelocity() > Constants.ShooterConstants.kShooterSpeedThreshold) {
                intakeAgitator.set(1);
            } else {
                intakeAgitator.set(0);
            }
        }

        if (shooting && slow) {
            // Only load balls if we are at speed
            if (shooterMotor.getEncoder().getVelocity() > Constants.ShooterConstants.kSlowShooterSpeedThreshold) {
                intakeAgitator.set(1);
            } else {
                intakeAgitator.set(0);
            }
        }
    }

    // set speed for testing
    public void testingSetMotorSpeed(double speed) {
        shooterMotor.set(speed);
        // shooterFeeder.set(speed * .75);
        intakeAgitator.set(speed * .75);
    }

    public void enableShooter() {
        shooting = true;
        shooterMotor.set(Constants.ShooterConstants.kFlywheelSpeed);
        // shooterFeeder.set(Constants.ShooterConstants.kFlywheelSpeed);
    }

    public void enableShooterSlow() {
        shooting = true;
        shooterMotor.set(0.2);
    }

    public void disableShooter() {
        shooting = false;
        shooterMotor.set(0);
        // shooterFeeder.set(0);
        intakeAgitator.set(0);
    }
}