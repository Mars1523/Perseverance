package frc.team1523.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1523.robot.Constants;

public class Shooter extends SubsystemBase {
    private final CANSparkMax shooterMotor = new CANSparkMax(8, CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax shooterFeeder = new CANSparkMax(10, CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax intakeAgitator = new CANSparkMax(11, CANSparkMax.MotorType.kBrushed);
    private boolean shooting = false;


    public Shooter(){
        intakeAgitator.setInverted(true);
        shooterFeeder.setInverted(true);
    }

    @Override
    public void periodic() {
        if (shooting) {
            // only leads balls if we are at speed
            if (shooterMotor.getEncoder().getVelocity() > Constants.ShooterConstants.kShooterSpeedThreshold) {
                intakeAgitator.set(1);
            } else {
                intakeAgitator.set(0);
            }
        }
    }

    public void testingSetMotorSpeed(double speed){
        shooterMotor.set(speed);
        shooterFeeder.set(speed * .75);
        intakeAgitator.set(speed * .75);

    }

    public void endableShooter(){
        shooting = true;
        shooterMotor.set(Constants.ShooterConstants.kFlywheelSpeed);
        shooterFeeder.set(Constants.ShooterConstants.kFlywheelSpeed);
    }
    public void disableShooter(){
        shooting = false;
        shooterMotor.set(0);
        shooterFeeder.set(0);
        intakeAgitator.set(0);
    }

//    public void setMotorSpeed(double speed) {
//        shooterMotor.set(speed * 0.5);
//    }
}