package frc.team1523.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private final SparkMax leftMotor = new SparkMax(12, MotorType.kBrushed);
    private final SparkMax rightMotor = new SparkMax(13, MotorType.kBrushed);

    public Climb() {
        SparkBaseConfig invertedConfig = new SparkMaxConfig().inverted(true);
        SparkBaseConfig noninvertedConfig = new SparkMaxConfig().inverted(false);


        rightMotor.configure(invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        leftMotor.configure(noninvertedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setMotorPower(double power) {
        leftMotor.set(power);
        rightMotor.set(power);
    }

    public void startClimbing() {
        setMotorPower(1);
    }

    public void stopClimbing() {
        setMotorPower(0);
    }

    public void startDescending() {
        setMotorPower(-1);
    }
}
