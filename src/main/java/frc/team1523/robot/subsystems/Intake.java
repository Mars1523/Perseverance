package frc.team1523.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import frc.team1523.robot.Constants;

public class Intake extends SubsystemBase {
    private final SparkMax intake = new SparkMax(7, MotorType.kBrushed);
    private final SparkMax wrist = new SparkMax(9, MotorType.kBrushed);
    private final Encoder wristEncoder = new Encoder(4, 5);
    private final PIDController controller = new PIDController(0.0256, 0, 0);

    public Intake() {
        Shuffleboard.getTab("Debug").add("Wrist PID", controller);
        Shuffleboard.getTab("Debug").add("Wrist Position", wristEncoder);
        
        SparkBaseConfig wristConfig = new SparkMaxConfig().inverted(false);
        wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }

    
    public double getWristSetpoint() {
        return controller.getSetpoint();
    }

    public void setWristSetpoint(double setpoint) {
        controller.setSetpoint(MathUtil.clamp(setpoint, 0, Constants.IntakeConstants.kWristRange));
    }

    @Override
    public void periodic() {
        wrist.set(controller.calculate(-wristEncoder.get()));
    }
}