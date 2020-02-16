/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.net.PortForwarder;

public class Vision extends SubsystemBase {
    private NetworkTable limelight;
    private NetworkTable openSight;

    // Variables for calculating distance
    private final double TARGET_HEIGHT = 98.25; // Outer port height above carpet in inches
    private final double LIMELIGHT_MOUNT_ANGLE = 63.5; // Angle that the Limelight is mounted at
    private final double LIMELIGHT_HEIGHT = 37.31; // Limelight height above the ground in inches

    public Vision() {
        PortForwarder.add(5800, "10.42.1.11", 5800);
        PortForwarder.add(5801, "10.42.1.11", 5801);
    public Vision() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        setPipeline(0);
        openSight = NetworkTableInstance.getDefault().getTable("OpenSight");
    }

    private void updateValidTarget() {
        if (hasTarget() || true) {
            setLastValidTargetTime();
        }
        if ((Timer.getFPGATimestamp() - lastValidTargetTime) < 5) {
            ledsOn();
            validTarget = true;
        } else {
            ledsOff();
            validTarget = false;
        }
    }

    public boolean getValidTarget() {
        return validTarget;
    }

    public void setLastValidTargetTime() {
        lastValidTargetTime = Timer.getFPGATimestamp();
    }

    public double getTargetY() {
    public double getTargetY() {
        return limelight.getEntry("ty").getDouble(0);
    }

    public double getTargetX() {
        return limelight.getEntry("tx").getDouble(0);
    }

    public double getInnerTargetX() {
        // TODO: Add adjustment for inner port
        return limelight.getEntry("tx").getDouble(0);
    }

    public boolean hasTarget() {
    public boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0) == 1;
    }

    public double getTargetArea() {
        return limelight.getEntry("ta").getDouble(0);
    }

    public double getTargetSkew() {
        return limelight.getEntry("ts").getDouble(0);
    }

    public double getPipelineLatency() {
        return limelight.getEntry("tl").getDouble(0);
    }

    public double getTargetShort() {
        return limelight.getEntry("tshort").getDouble(0);
    }

    public double getTargetLong() {
        return limelight.getEntry("tlong").getDouble(0);
    }

    public double getHorizontalSidelength() {
        return limelight.getEntry("thor").getDouble(0);
    }

    public double getVerticalSidelength() {
        return limelight.getEntry("tvert").getDouble(0);
    }

    public double getPipeline() {
        return limelight.getEntry("getpipe").getDouble(0);
    }

    public void ledsOn() {
        limelight.getEntry("ledMode").setNumber(3);
    }

    public void ledsOff() {
        limelight.getEntry("ledMode").setNumber(1);
    }

    public void setPipeline(int pipeline) {
        limelight.getEntry("pipeline").setNumber(pipeline);
    }

    public double getTargetDistance() {
        double[] distances = new double[5];
        for (int i = 0; i < distances.length; i++) {
            double angleToTarget = getTargetY();
            double inches = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(LIMELIGHT_MOUNT_ANGLE + angleToTarget);
            distances[i] = inches / 12;
        }

        double distance = computeMode(distances);

        // Adjust distance based on pipeline (zoom level)
        int pipeline = (int)getPipeline();
        switch (pipeline) {
            case 1:
                // TODO
                break;

            case 2:
                // TODO
                break;

            case 3:
                // TODO
                break;
        }

        return distance;
    }

    private double computeMode(double[] data) {
        // Compute mode
        double[] counts = new double[data.length]; // Does this need to be zero initialized?
        for (int i = 0; i < data.length; i++)
        {
            for (int j = 0; j < data.length; i++)
            {
                if (data[i] == data[j])
                {
                    counts[i]++;
                }
            }
        }

        int highestIndex = 0;
        double previousHigh = 0;
        for (int i = 0; i < counts.length; i++)
        {
            if (counts[i] > previousHigh)
            {
                highestIndex = i;
                previousHigh = counts[i];
            }
        }

        return data[highestIndex]; // Final distance in feet
    }

    public double getPowerCellX() {
        double xOffset = openSight.getEntry("ball").getDoubleArray(new double[] {0, 0})[0];

        // TODO: Use xOffset to return a positive value when target is the right and vice versa for right

        return xOffset;
    }

    public boolean hasPowerCell() {
        return openSight.getEntry("found").getBoolean(false);
    }

    /*
    public void openSightInit() {
        // Seems to be necessary to get OpenSight cam to show up in Shuffleboard
        // CameraServer.getInstance().addAxisCamera("opensight", "opensight.local");
        CameraServer.getInstance().addServer("opensight.local");
        // TODO: Fix this?
        PortForwarder.add(6000, "opensight.local", 5800);
    }
     */

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Limelight Target Distance", getTargetDistance());
        SmartDashboard.putNumber("Limelight Pipeline", getPipeline());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateSmartDashboard();
        updateValidTarget();
    }
}
