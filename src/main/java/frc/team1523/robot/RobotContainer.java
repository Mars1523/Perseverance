package frc.team1523.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
    private final Turret turret = new Turret();
    private final ColorWheel colorWheel = new ColorWheel();
    private final Leds leds = new Leds();


    public RobotContainer() {
        configureButtonBindings();

        chooser.setDefaultOption("Default Auto", kDefaultAuto);
        chooser.addOption("My Auto", kCustomAuto);
        Shuffleboard.getTab("Drive").add("Auto choices", chooser);

        drivetrain.setDefaultCommand(new RunCommand(() ->
                drivetrain.drive(-primaryController.getY(GenericHID.Hand.kLeft),
                        primaryController.getX(GenericHID.Hand.kLeft) * 0.8),
                drivetrain));

        intake.setDefaultCommand(new RunCommand(() -> {
            intake.setIntakeSpeed(primaryController.getY(GenericHID.Hand.kRight));
            intake.setWristSpeed(-alternateController.getY(GenericHID.Hand.kRight));
        }, intake));

        colorWheel.setDefaultCommand(new RunCommand(() -> {
            colorWheel.setExtendSpeed(alternateController.getX(GenericHID.Hand.kRight));
            colorWheel.setSpinySpeed(alternateController.getX(GenericHID.Hand.kLeft));
        }, colorWheel));
    }

    private void configureButtonBindings() {
        new JoystickButton(primaryController, XboxController.Button.kBumperRight.value)
                .whenPressed(new InstantCommand(() -> shooter.setMotorSpeed(1)))
                .whenReleased(new InstantCommand(() -> shooter.setMotorSpeed(0)));
    }

    public Command getAutonomousCommand() {
        String m_autoSelected = chooser.getSelected();
        switch (m_autoSelected) {
            case kCustomAuto:
                return new InstantCommand();
            case kDefaultAuto:
            default:
                return new InstantCommand();
        }
    }
}