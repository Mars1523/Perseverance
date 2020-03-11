package frc.team1523.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1523.robot.Constants;

public class Lift extends SubsystemBase {
    private final CANSparkMax liftMotor = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushed);
    private final Encoder liftEncoder = new Encoder(0, 1);

    public Lift() {
        Shuffleboard.getTab("Debug").add("Lift Encoder", liftEncoder);
        liftMotor.setInverted(true);
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
