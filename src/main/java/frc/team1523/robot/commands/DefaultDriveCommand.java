package frc.team1523.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1523.robot.Constants;
import frc.team1523.robot.subsystems.Drivetrain;

public class DefaultDriveCommand extends Command {

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(2.5);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
    private XboxController primaryController;
    private Drivetrain drivetrain;

    public DefaultDriveCommand(XboxController primaryController, Drivetrain drivetrain) {
        addRequirements(drivetrain);
        this.primaryController = primaryController;
        this.drivetrain = drivetrain;
    }

    static double deadband(double value, double limit) {
        if (Math.abs(value) > limit) {
            return value;
        } else {
            return 0;
        }
    }

    @Override
    public void execute() {
        if (true) {
            // Get the x speed. We are inverting this because Xbox controllers return
            // negative values when we push forward.
            double rawX = primaryController.getLeftX();

            final double rot = rotLimiter.calculate(deadband(rawX, .1)
                    * Constants.DriveConstants.kMaxAngularSpeed);

            // Get the rate of angular rotation. We are inverting this because we want a
            // positive value when we pull to the left (remember, CCW is positive in
            // mathematics). Xbox controllers return positive values when you pull to
            // the right by default.
            double rawY = primaryController.getRightY();
            final double xSpeed = speedLimiter.calculate(deadband(rawY, .1) 
                    * Constants.DriveConstants.kMaxSpeed);

            drivetrain.fancyDrive(xSpeed, rot);

        } else {
            drivetrain.boringDrive(-primaryController.getRightY(),
                    primaryController.getLeftX());
        }
    }
}
