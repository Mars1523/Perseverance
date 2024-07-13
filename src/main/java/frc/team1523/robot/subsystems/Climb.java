package frc.team1523.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private final CANSparkMax leftMotor = new CANSparkMax(12, CANSparkMax.MotorType.kBrushed);
    private final CANSparkMax rightMotor = new CANSparkMax(13, CANSparkMax.MotorType.kBrushed);

    public Climb() {
        rightMotor.setInverted(true);
        leftMotor.setInverted(false);
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
