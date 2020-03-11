package frc.team1523.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.team1523.robot.subsystems.Drivetrain;


public class TurnCommand extends PIDCommand {
    private final Drivetrain drivetrain;

    public TurnCommand(double targetAngle, Drivetrain drivetrain) {
        super(new PIDController(0.04, 0.001, 0.0),
                drivetrain::getAngle,
                targetAngle,
                (double turn) -> drivetrain.boringDrive(0, turn),
                drivetrain);

        this.drivetrain = drivetrain;
        getController().setTolerance(0.2);
    }

    @Override
    public void initialize() {
        drivetrain.zeroSensors();
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}