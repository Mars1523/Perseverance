package frc.team1523.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team1523.robot.commands.DefaultDriveCommand;
import frc.team1523.robot.commands.DeployLift;
import frc.team1523.robot.commands.LimelightTurnToTarget;
import frc.team1523.robot.commands.TurnCommand;
import frc.team1523.robot.subsystems.*;

public class RobotContainer {
    // Auto chooser, add additional autos here
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private final SendableChooser<String> chooser = new SendableChooser<>();

    private final XboxController primaryController = new XboxController(0);
    private final XboxController alternateController = new XboxController(1);


    // Create subsystems
    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intake = new Intake();
    private final Limelight limelight = new Limelight();
    private final Shooter shooter = new Shooter();
    private final Leds leds = new Leds();
    private final Lift lift = new Lift();
    private final Climb climb = new Climb();

    private final NetworkTableEntry climbReversalEnabledEntry = Shuffleboard.getTab("Debug")
            .add("Climb whinch reversal", false)
            .getEntry();

    public RobotContainer() {
        configureButtonBindings();

        climbReversalEnabledEntry.setBoolean(false);

        chooser.setDefaultOption("Default Auto", kDefaultAuto);
        chooser.addOption("My Auto", kCustomAuto);
        Shuffleboard.getTab("Drive").add("Auto choices", chooser);

        Shuffleboard.getTab("Debug").add(new PowerDistributionPanel());

        drivetrain.setDefaultCommand(new DefaultDriveCommand(primaryController, drivetrain));

        intake.setDefaultCommand(new RunCommand(() -> {
            intake.setIntakeSpeed(-alternateController.getY(GenericHID.Hand.kLeft));
            double raw = -alternateController.getY(GenericHID.Hand.kRight);
            double wrist = Math.copySign(Math.pow(raw, 2), raw);

            intake.setWristSetpoint(intake.getWristSetpoint() + (wrist * 6));
        }, intake));

//        shooter.setDefaultCommand(new RunCommand(() -> {
//            shooter.testingSetMotorSpeed(alternateController.getY(GenericHID.Hand.kRight));
//        }, shooter));

        CameraServer.getInstance().startAutomaticCapture();
    }

    private void configureButtonBindings() {
        new JoystickButton(primaryController, XboxController.Button.kBumperRight.value)
                .whenPressed(new InstantCommand(shooter::enableShooter))
                .whenReleased(new InstantCommand(shooter::disableShooter));

        new JoystickButton(primaryController, XboxController.Button.kA.value)
                .whileActiveContinuous(new RunCommand(drivetrain::alarm));

        new JoystickButton(primaryController, XboxController.Button.kStart.value)
                .whenPressed(new InstantCommand(limelight::enableLeds));

        new JoystickButton(primaryController, XboxController.Button.kBack.value)
                .whenPressed(new InstantCommand(limelight::disableLeds));

        new JoystickButton(primaryController, XboxController.Button.kB.value)
                .whileActiveContinuous(new LimelightTurnToTarget(drivetrain, limelight));

        new JoystickButton(alternateController, XboxController.Button.kX.value)
                .whileActiveContinuous(new DeployLift(lift, false));

        new JoystickButton(alternateController, XboxController.Button.kY.value)
                .whileActiveContinuous(new DeployLift(lift, true));

        new JoystickButton(alternateController, XboxController.Button.kB.value)
                .whenPressed(new InstantCommand(climb::startClimbing, climb))
                .whenReleased(new InstantCommand(climb::stopClimbing, climb));

        new JoystickButton(alternateController, XboxController.Button.kA.value)
                .whenPressed(new InstantCommand(() -> {
                    if (climbReversalEnabledEntry.getBoolean(false)) {
                        climb.startDescending();
                    }
                }, climb))
                .whenReleased(new InstantCommand(climb::stopClimbing, climb));


    }

    public Command getAutonomousCommand() {
        String m_autoSelected = chooser.getSelected();
        switch (m_autoSelected) {
            case kCustomAuto:
                return new TurnCommand(90, drivetrain);
            case kDefaultAuto:
            default:
                return new InstantCommand();
        }
    }
}