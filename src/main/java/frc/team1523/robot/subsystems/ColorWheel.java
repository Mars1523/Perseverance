package frc.team1523.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorWheel extends SubsystemBase {
    private final Spark extend = new Spark(0);
    private final Spark spinyWheel = new Spark(1);

    public void setExtendSpeed(double speed) {
        extend.set(speed);
    }

    public void setSpinySpeed(double speed) {
        spinyWheel.set(speed);
    }
}
