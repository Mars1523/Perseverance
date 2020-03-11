/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1523.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import static frc.team1523.robot.Constants.LimelightConstants.*;

public class Limelight extends SubsystemBase {
    // Create variables for the different values given from the limelight
    private double xOffset; // Positive values mean that target is to the right of the camera; negative
    // values mean target is to the left. Measured in degrees
    private double yOffset; // Positive values mean that target is above the camera; negative values mean
    // target is below. Measured in degrees
    private double targetArea; // Returns a value of the percentage of the image the target takes
    private double targetValue; // Sends 1 if a target is detected, 0 if none are present
    // Create a network table for the limelight
    private MedianFilter distanceFilter = new MedianFilter(35);


    private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight data");
    private NetworkTableEntry limelightXOffsetEntry = limelightTab
            .add("Limelight X Offset", xOffset)
            .getEntry();
    private NetworkTableEntry limelightYOffsetEntry = limelightTab
            .add("Limelight Y Offset", yOffset)
            .getEntry();
    private NetworkTableEntry limelightAreaPercentageEntry = limelightTab
            .add("Limelight Area Percentage", targetArea)
            .getEntry();
    private NetworkTableEntry limelightTargetCenteredEntry = limelightTab
            .add("Target Centered", isTargetCentered())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();
    private NetworkTableEntry limelightTargetDetectedEntry = limelightTab
            .add("Target Detected", isTargetDetected())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();
    private NetworkTableEntry limelightLedEntry = NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("ledMode");

//    private NetworkTableEntry limelightDistanceEntry = Shuffleboard.getTab("Limelight data")
//            .add("Distance (INCHES)", limelightDistance())
//            .getEntry();

    public Limelight() {
        // Reset the default settings and pipelines to the Limelight
        limelightTable.getEntry("pipeline").setNumber(0);
    }

    /**
     * Returns a value of the offset on the x-axis of the camera to the target in
     * degrees. Negative values mean the target is to the left of the camera
     */
    public double getXOffset() {
        return xOffset;
    }

    /**
     * Returns true if a target is detected
     */
    public boolean isTargetDetected() {
        return (targetValue > 0.0);
    }

    /**
     * Returns true if the target is within a range of the center crosshair of the
     * camera
     */
    public boolean isTargetCentered() {
        return ((xOffset > -1.5) && (xOffset < 1.5) && (xOffset != 0.0));
    }

    /**
     * Calculates the total angle by adding the mounting angle with the y-axis
     * offset angle of the limelight in degrees
     */
    public double limelightAngle() {
        throw new UnsupportedOperationException();
//        return (kLimelightAngle + yOffset);
    }

    /**
     * Return the distance from the limelight to the target in inches (floor
     * distance)
     */
    public double limelightDistance() {
        return (kPortHeight - kLimelightHeight) / Math.tan(Math.toRadians(kLimelightAngle + yOffset) + kLimelightOffset);
    }

    public double filteredLimelightDistance() {
        return distanceFilter.calculate(limelightDistance());
    }
    /**
     * Returns the value of the pipeline from the network table
     *
     * @return pipelineValue
     */
    public double getPipeline() {
        NetworkTableEntry pipeline = limelightTable.getEntry("Pipeline");
        return pipeline.getDouble(0.0);
    }

    /**
     * Chooses which pipeline to use on the limelight and prevents invalid values
     * from being sent
     *
     * @param pipeline Which pipeline to use on the limelight (0-9)
     */
    public void setPipeline(int pipeline) {
        int clampedPipeline = MathUtil.clamp(pipeline, 0, 9);
        limelightTable.getEntry("Pipeline").setValue(clampedPipeline);
    }

    /**
     * Enable the leds on the limelight
     */
    public void enableLeds() {
        limelightLedEntry.setNumber(LimelightConstants.kLedEnabled);
    }

    /**
     * Disabled the leds on the limelight
     */
    public void disableLeds() {
        limelightLedEntry.setNumber(LimelightConstants.kLedDisabled);
    }

    /**
     * Blink the leds on the limelight
     */
    public void blinkLeds() {
        limelightLedEntry.setNumber(LimelightConstants.kLedBlink);
    }

    public void updateLimelight() {
        // Updates the values of the limelight on the network table
        xOffset = limelightTable.getEntry("tx").getDouble(0.0);
        yOffset = limelightTable.getEntry("ty").getDouble(0.0);
        targetArea = limelightTable.getEntry("ta").getDouble(0.0);
        targetValue = limelightTable.getEntry("tv").getDouble(0.0);
    }

    public void log() {
        // Updates the SmartDashboard with limelight values
        limelightXOffsetEntry.setNumber(xOffset);
        limelightYOffsetEntry.setNumber(yOffset);

        limelightAreaPercentageEntry.setNumber(targetArea);
        limelightTargetCenteredEntry.setBoolean(isTargetCentered());
        limelightTargetDetectedEntry.setBoolean(isTargetDetected());
//        limelightDistanceEntry.setNumber(limelightDistance());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateLimelight();
        log();
    }

    public static final class LimelightConstants {
        public static final int kLedDisabled = 1;
        public static final int kLedBlink = 2;
        public static final int kLedEnabled = 3;
    }
}