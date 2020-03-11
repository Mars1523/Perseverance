package frc.team1523.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team1523.robot.Constants;

public class Intake extends PIDSubsystem {
    private final CANSparkMax intake = new CANSparkMax(7, CANSparkMax.MotorType.kBrushed);
    private final CANSparkMax wrist = new CANSparkMax(9, CANSparkMax.MotorType.kBrushed);

    private final Encoder wristEncoder = new Encoder(4, 5);

    public Intake() {
        super(new PIDController(0.0256, 0.0, 0.0));
        enable();
        Shuffleboard.getTab("Debug").add("Wrist PID", getController());
        Shuffleboard.getTab("Debug").add("Wrist Position", wristEncoder);
        wrist.setInverted(false);
    }

    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }

    public double getWristSetpoint() {
        return getController().getSetpoint();
    }

    public void setWristSetpoint(double setpoint) {
        setSetpoint(MathUtil.clamp(setpoint, 0, Constants.IntakeConstants.kWristRange));
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        wrist.set(output);
    }

    @Override
    protected double getMeasurement() {
        return -wristEncoder.get();
    }
}