package frc.team1523.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1523.robot.Constants;

public class Lift extends SubsystemBase {
    private final SparkMax liftMotor = new SparkMax(14, MotorType.kBrushed);
    private final Encoder liftEncoder = new Encoder(0, 1);

    public Lift() {
        Shuffleboard.getTab("Debug").add("Lift Encoder", liftEncoder);
        SparkBaseConfig invertedConfig = new SparkMaxConfig().inverted(true);
        liftMotor.configure(invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setLiftPower(double power) {
        if (power > 0 && liftEncoder.get() > Constants.LiftConstants.kLiftMax) {
            liftMotor.set(0);
        } else {
            liftMotor.set(power);
        }
    }

    public double getEncoderValue() {
        return liftEncoder.get();
    }
}
