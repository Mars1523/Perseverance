package frc.team1523.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team1523.robot.Constants;
import frc.team1523.robot.subsystems.Drivetrain;
import frc.team1523.robot.subsystems.Limelight;

public class LimelightTurnToTarget extends CommandBase {
    private final PIDController turnPid;
    private final PIDController distancePid;
    private final Drivetrain drivetrain;
    private final Limelight limelight;

    public LimelightTurnToTarget(Drivetrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        turnPid = new PIDController(0.0525, 0.009, 0.0001);
        turnPid.setSetpoint(0);
        distancePid = new PIDController(0.0225, 0.0, 0.0001);
        distancePid.setSetpoint(-70);

        Shuffleboard.getTab("Debug").add("Limelight turn", turnPid);


    }

    @Override
    public void execute() {
        if (limelight.isTargetDetected()) {
            double rotOut = turnPid.calculate(limelight.getXOffset());

            double distance = limelight.filteredLimelightDistance();
            double distancePidOut = MathUtil.clamp(distancePid.calculate(distance),
                    -.4 * Constants.DriveConstants.kMaxSpeed,
                    .4 * Constants.DriveConstants.kMaxSpeed);

            System.out.println(distancePidOut);
            drivetrain.fancyDrive(-(distancePidOut), -rotOut);


        } else {
            drivetrain.fancyDrive(0, 0);
        }
    }

    @Override
    public void initialize() {
        turnPid.reset();
        distancePid.reset();
    }
}

//class LimelightTurnToTarget2 extends PIDCommand {
//    public LimelightTurnToTarget2(Drivetrain drivetrain, Limelight limelight) {
////        super(new PIDController(0.0095, 0.0025, 0.0001),
//        super(new PIDController(0.0525, 0.009, 0.0001),
////        super(new PIDController(0.052, 0.009, 0.0004),
//                () -> {
//                    if (limelight.isTargetDetected()) {
//                        return limelight.getXOffset();
//                    } else {
//                        return 0;
//
//                    }
//                },
//                0,
//                (double turn) -> {
//                    System.out.println(turn);
//                    drivetrain.fancyDrive(0, -turn + Math.copySign(.16, -turn));//* (Math.PI/2));
//                },
//                drivetrain);
//        Shuffleboard.getTab("Debug").add("Limelight turn", getController());
//        getController().setTolerance(2);
//    }
//
//    @Override
//    public void initialize() {
//        getController().reset();
//    }
//}
