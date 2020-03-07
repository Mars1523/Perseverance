package frc.team1523.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
    private final CANSparkMax intake = new CANSparkMax(7, CANSparkMax.MotorType.kBrushed);
    private final CANSparkMax wristcontroller = new CANSparkMax(8, CANSparkMax.MotorType.kBrushed);

    public Intake(){
        wristcontroller.setInverted(true);
    }

//    public Intake(){
//        super(new PIDController(0.0256, 0.0, 0.0));
//    }

    public void setWristSpeed(double speed) {
        wristcontroller.set(speed);
    }

    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }

//    public void setWristSetpoint(double setpoint){
//        setSetpoint(MathUtil.clamp(setpoint, 0, kWristRange));
//    }
//    @Override
//    protected void useOutput(double output, double setpoint){
//        wristcontroller.set(output);
//    }
//    @Override
//    protected  double getMeasurement(){
//        return 0;
//    }
}