/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
Subsystem for interacting with the Limelight and OpenSight vision systems
 */

public class Vision extends SubsystemBase {
    // Variables for calculating distance
    private final double TARGET_HEIGHT = 98.25; // Outer port height above carpet in inches
    private final double LIMELIGHT_MOUNT_ANGLE = 18.48; // Angle that the Limelight is mounted at
    private final double LIMELIGHT_HEIGHT = 37.31; // Limelight height above the ground in inches

    private final double MIN_TARGET_DISTANCE = 1;
    private final double INNER_PORT_SLOPE = 1;
    private final double INNER_PORT_OFFSET = 1;

    private final double HORIZONTAL_TARGET_PIXEL_WIDTH = 1;
    private final double HORIZONTAL_TARGET_PIXEL_THRESHOLD = 1;
    private final double VERTICAL_TARGET_PIXEL_WIDTH = 1;
    private final double VERTICAL_TARGET_PIXEL_THRESHOLD = 1;

    // NetworkTables for reading vision data
    private final NetworkTable oak_d_goal;
    private final NetworkTable oak_1_intake;

    // Subsystems that will be controlled based on vision data
    private final DriveTrain m_driveTrain;
    private final Turret m_turret;
    double[] distances = new double[5];
    double[] counts = new double[5];
    int index = 0;
    // Filters to prevent target values from oscillating too much
    SlewRateLimiter targetXFilter = new SlewRateLimiter(20);
    SlewRateLimiter innerTargetXFilter = new SlewRateLimiter(20);

    private boolean resetPose;
    private double lastValidTargetTime;
    private boolean validTarget;

    public Vision(DriveTrain driveTrain, Turret turret) {
        m_driveTrain = driveTrain;
        m_turret = turret;

        // Init vision NetworkTables
        oak_d_goal = NetworkTableInstance.getDefault().getTable("OAK-D_Goal");
        oak_1_intake = NetworkTableInstance.getDefault().getTable("OAK-1_Intake");

        //initShuffleboard();
    }

    private void updateValidTarget() {
        // Determine whether the limelight has detected a valid target and not a random reflection
        // If the target is seen for a specific amount of time it is marked as valid
        if(hasTarget()) {
            setLastValidTargetTime();
        }
        if((Timer.getFPGATimestamp() - lastValidTargetTime) < 3) {
            validTarget = true;
        } else {
            validTarget = false;
        }
    }

    public boolean getValidTarget() {
        return validTarget;
    }

    public void setLastValidTargetTime() {
        lastValidTargetTime = Timer.getFPGATimestamp();
    }

    // Limelight interaction functions
    public double getTargetY() {
        return oak_d_goal.getEntry("ty").getDouble(0);
    }

    public double getTargetX() {
        return oak_d_goal.getEntry("tx").getDouble(0);
    }

    public double getTargetDistance() {
        return oak_d_goal.getEntry("tz").getDouble(0);
    }

    public double getFilteredTargetX() {
        return targetXFilter.calculate(getTargetX());
    }


    public double getSmartTargetX() {
        if(getTargetDistance() > MIN_TARGET_DISTANCE) {
            double xDistance = Units.metersToFeet(m_driveTrain.getRobotPose().getTranslation().getX());
            double yDistance = Math.abs(Units.metersToFeet(m_driveTrain.getRobotPose().getTranslation().getY()));

            double maxYDistance = INNER_PORT_SLOPE * xDistance + INNER_PORT_OFFSET;

            if(yDistance < maxYDistance) {
                xDistance += 29.25 / 12.0;
                return innerTargetXFilter.calculate(Math.signum(getFilteredTargetX()) * Units.radiansToDegrees(Math.atan(xDistance / yDistance)));
            }
        }

        return getFilteredTargetX();
    }

    private void resetPoseByVision() {
        if(! resetPose) {
            if((Math.abs(getHorizontalSidelength() - HORIZONTAL_TARGET_PIXEL_WIDTH) < HORIZONTAL_TARGET_PIXEL_THRESHOLD) &&
                    (Math.abs(getVerticalSidelength() - VERTICAL_TARGET_PIXEL_WIDTH) < VERTICAL_TARGET_PIXEL_THRESHOLD)) {
                double targetRadians = Units.degreesToRadians(m_turret.getFieldRelativeAngle());
                double xDistance = Math.abs(Math.cos(targetRadians)) * getTargetDistance();
                double yDistance = - Math.signum(getFilteredTargetX()) * Math.abs(Math.sin(targetRadians)) * getTargetDistance();

                m_driveTrain.resetOdometry(new Pose2d(xDistance, yDistance, new Rotation2d()),
                        Rotation2d.fromDegrees(m_driveTrain.getHeading()));

                resetPose = true;
            }
        } else if(resetPose && ! hasTarget()) {
            resetPose = false;
        }
    }

    // More Limelight interaction functions

    public boolean hasTarget() {
        return oak_d_goal.getEntry("tv").getDouble(0) == 1;
    }

    public double getTargetArea() {
        return oak_d_goal.getEntry("ta").getDouble(0);
    }

    public double getTargetSkew() {
        return oak_d_goal.getEntry("ts").getDouble(0);
    }

    public double getPipelineLatency() {
        return oak_d_goal.getEntry("tl").getDouble(0);
    }

    public double getTargetShort() {
        return oak_d_goal.getEntry("tshort").getDouble(0);
    }

    public double getTargetLong() {
        return oak_d_goal.getEntry("tlong").getDouble(0);
    }

    public double getHorizontalSidelength() {
        return oak_d_goal.getEntry("thor").getDouble(0);
    }

    public double getVerticalSidelength() {
        return oak_d_goal.getEntry("tvert").getDouble(0);
    }

    
    // For Shoot on the Move, gets horizontal angle on field to target
    public double getHorizontalAngleToTarget() {
        return getTargetX();
            // TODO: Figure out what to add/subtract if we're zoomed in
    }

    // Used to find the most common value to provide accurate target data
    private double computeMode(double[] data) {
        // Compute mode
        this.counts = new double[data.length];
        for(int i = 0; i < data.length; i++) {
            for(double datum : data) {
                if(data[i] == datum) {
                    this.counts[i]++;
                }
            }
        }

        int highestIndex = 0;
        double previousHigh = 0;
        for(int i = 0; i < this.counts.length; i++) {
            if(this.counts[i] > previousHigh) {
                highestIndex = i;
                previousHigh = this.counts[i];
            }
        }

        return data[highestIndex]; // Final distance in feet
    }

    // Read ball position data from OAK-1_Intake
    public double getPowerCellX() {
        double[] nullValue = {-99};
        var values = oak_1_intake.getEntry("ta").getDoubleArray(nullValue);

        if(values[0] == -99) {
            return 0;
        } else {
            return values[0];
        }
    }

    public boolean hasPowerCell() {
        return oak_1_intake.getEntry("tv").getDouble(0) == 1;
    }

    private void initShuffleboard() {
        // Unstable. Don''t use until WPILib fixes this
        Shuffleboard.getTab("Turret").addBoolean("Vision Valid Output", this :: getValidTarget);
        Shuffleboard.getTab("Turret").addNumber("Vision Target X", this :: getFilteredTargetX);

    }

    public void updateSmartDashboard() {
        SmartDashboard.putBoolean("OAK-D_Goal Has Target", hasTarget());
        SmartDashboard.putNumber("OAK-D_Goal Target X", getTargetX());
        SmartDashboard.putNumber("OAK-D_Goal Target Y", getTargetY());
        SmartDashboard.putNumber("OAK-D_Goal Target Distance", getTargetDistance());

        SmartDashboardTab.putBoolean("Turret", "Vision Valid Output", getValidTarget());
        SmartDashboardTab.putNumber("Turret", "Vision Target X", getFilteredTargetX());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateSmartDashboard();
        updateValidTarget();

        //resetPoseByVision();
    }
}
