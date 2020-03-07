package frc.team1523.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivetrain extends SubsystemBase {
    private final WPI_TalonFX leftFront = new WPI_TalonFX(2);
    private final WPI_TalonFX rightRear = new WPI_TalonFX(3);
    private final WPI_TalonFX leftRear = new WPI_TalonFX(4);
    private final WPI_TalonFX rightFront = new WPI_TalonFX(5);
    private final AHRS navx = new AHRS();

    SpeedController leftMotors = new SpeedControllerGroup(leftFront, leftRear);
    SpeedController rightMotors = new SpeedControllerGroup(rightFront, rightRear);
    private final DifferentialDrive robotDrive = new DifferentialDrive(leftMotors, rightMotors);

    public Drivetrain() {
        leftFront.configOpenloopRamp(.2);
        rightRear.configOpenloopRamp(.2);
        leftRear.configOpenloopRamp(.2);
        rightFront.configOpenloopRamp(.2);
        leftFront.setNeutralMode(NeutralMode.Brake);
        rightRear.setNeutralMode(NeutralMode.Brake);
        leftRear.setNeutralMode(NeutralMode.Brake);
        rightFront.setNeutralMode(NeutralMode.Brake);
    }

    public double getAngle(){
        return navx.getAngle();
    }

    public void xeroSensors() {
        navx.zeroYaw();
    }

    public void drive(double xSpeed, double rotation) {
        robotDrive.arcadeDrive(xSpeed, rotation);
    }
}