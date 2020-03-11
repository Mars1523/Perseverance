package frc.team1523.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team1523.robot.Constants;
import frc.team1523.robot.subsystems.Lift;

public class DeployLift extends CommandBase {
    private final Lift lift;
    private final boolean reverse;

    public DeployLift(Lift lift, boolean reverse) {
        this.lift = lift;
        this.reverse = reverse;
    }

    @Override
    public void execute() {
        if (reverse) {
            lift.setLiftPower(-.8);
        } else {
            lift.setLiftPower(1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        lift.setLiftPower(0);
    }

    @Override
    public boolean isFinished() {
        if (reverse) {
            return false;
        }
        return lift.getEncoderValue() > Constants.LiftConstants.kLiftMax;
    }
}
