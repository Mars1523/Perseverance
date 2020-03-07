package frc.team1523.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;

@SuppressWarnings("unused")
public class Leds{
    private SerialPort serial;
    private LedPattern last;
    private boolean warning = false;

    public Leds() {
        // Don't explode if there is nothing plugged in
        try {
            serial = new SerialPort(9600, SerialPort.Port.kUSB);
        } catch (Exception ignored) {
            serial = null;
        }

    }

    public void startAllianceFader() {
        if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Red) {
            writePattern(LedPattern.RedFader);
        } else {
            writePattern(LedPattern.BlueFader);
        }
    }

    public void startPulsingWarning(boolean enabled) {
        if (warning != enabled) {
            if (enabled) {
                writePatternForgetful(LedPattern.PulsingWarning);
            } else {
                writePatternForgetful(last);
            }
        }
        warning = enabled;
    }

    private void writePattern(LedPattern pattern) {
        last = pattern;
        writeByte((byte) pattern.value);
    }

    private void writePatternForgetful(LedPattern pattern) {
        writeByte((byte) pattern.value);
    }

    private void writeByte(byte data) {
        if (serial != null) {
            serial.write(new byte[]{data}, 1);
        }
    }

    public enum LedPattern {
        Off(0),
        RedFader(1),
        BlueFader(2),
        PulsingWarning(4),
        Rainbow1(7),
        Rainbow2(8);

        public final int value;

        LedPattern(int value) {
            this.value = value;
        }
    }
}
