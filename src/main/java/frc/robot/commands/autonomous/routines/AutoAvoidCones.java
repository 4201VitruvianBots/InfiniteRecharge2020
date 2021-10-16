/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class AutoAvoidCones extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain m_driveTrain;
    private final DoubleSupplier m_throttleOutput, m_turnOutput;
    private double currentThrottleOutput, currentTurnOutput;
    private final Translation2d[] m_conePoses;
    private final BooleanSupplier m_override;

    private final double tolerance = Units.inchesToMeters(6);
    private final double robotLength = 0.838 + tolerance;
    private final double robotWidth = 0.673 + tolerance;

    private final double coneRadius = Units.inchesToMeters(6);
    /* Robot Points:
           ^: Front of the robot
             0-------1
             |   ^   |
             3-------2
         */
    Translation2d[] robotCornerPoses = new Translation2d[4];

    public AutoAvoidCones(DriveTrain driveTrain, DoubleSupplier throttleOutput, DoubleSupplier turnOutput, BooleanSupplier override, Translation2d[] conePoses) {
        m_driveTrain = driveTrain;
        m_throttleOutput = throttleOutput;
        m_turnOutput = turnOutput;
        m_conePoses = conePoses;
        m_override = override;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        currentThrottleOutput = m_throttleOutput.getAsDouble();
        currentTurnOutput = m_turnOutput.getAsDouble();
        if (aboutToHitCone() && !m_override.getAsBoolean()) {
            m_driveTrain.setMotorTankDrive(0, 0);
        } else {
            m_driveTrain.setMotorArcadeDrive(currentThrottleOutput, currentTurnOutput);
        }
    }

    private boolean aboutToHitCone() {
        Pose2d robotPose = m_driveTrain.getRobotPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double cos = robotPose.getRotation().getCos();
        double sin = robotPose.getRotation().getSin();

        Translation2d[] robotCornerPoses = {
            new Translation2d(robotLength / 2.0, robotLength / 2.0),
            new Translation2d(robotLength / 2.0, -robotLength / 2.0),
            new Translation2d(-robotLength / 2.0, -robotLength / 2.0),
            new Translation2d(-robotLength / 2.0, robotLength / 2.0)
        };
        for (int i = 0; i < 4; i++) {
            robotCornerPoses[i] = robotCornerPoses[i].rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
        }

        for (Translation2d cone : m_conePoses) { 
            Translation2d adjustedConePose = cone.minus(robotPose.getTranslation()).rotateBy(robotPose.getRotation().times(-1)).plus(robotPose.getTranslation());

            boolean coneToleft = adjustedConePose.getX() < robotPose.getX();
            if (
                coneToleft == currentThrottleOutput < 0 &&
                (adjustedConePose.getX() + coneRadius > robotPose.getX() - robotLength / 2) && 
                (adjustedConePose.getX() - coneRadius < robotPose.getX() + robotLength / 2) &&
                adjustedConePose.getY() + coneRadius > robotPose.getY() - robotWidth / 2 && 
                adjustedConePose.getY() - coneRadius < robotPose.getY() + robotWidth / 2
                ) {
                    SmartDashboardTab.putBoolean("DriveTrain", "Will hit Cone", true);
                    return true;
                }
        }
        SmartDashboardTab.putBoolean("DriveTrain", "Will hit Cone", false);
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
