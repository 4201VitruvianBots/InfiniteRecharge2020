package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.autonomous.SmartdashboardCommand;
import frc.robot.commands.autonomous.TurnInPlace;
import frc.robot.commands.autonomous.VitruvianRamseteCommand;
import frc.robot.commands.drivetrain.SetDriveNeutralMode;
import frc.robot.commands.drivetrain.SetDriveShifters;
import frc.robot.commands.drivetrain.SetOdometry;
import frc.robot.commands.intake.SetIntakeSpeed;
import frc.robot.commands.intake.SetIntakePiston;
import frc.robot.commands.intake.SetIntakeStates;
import frc.robot.constants.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimulationShoot;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;
import java.util.ArrayList;
import java.util.List;
/**
 * Generates each trajectory right before running it instead of all at once
 * in order to adjust for robot's actual vs. expected position disparities
 */
public class AutoNewBarrelTest extends CommandBase {
    private Pose2d currentPose;
    int[][] endPointsRaw = {
        {150,90,0},
        {176,45,-120},
        {124,45,120},
        {150,90,0},
        {240,90,0},
        {270,120,90},
        {210,120,-90},
        {285,34,0},
        {330,60,90},
        {285,90,180},
        {30,90,180}
    };

    Pose2d[] endPoints = new Pose2d[endPointsRaw.length];

    private int index = 0;
    private DriveTrain m_driveTrain;
    private FieldSim m_fieldSim;
    private TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(10), Units.feetToMeters(10));;
    private Trajectory trajectory;
    private VitruvianRamseteCommand command;
    private boolean complete = false;

    public AutoNewBarrelTest(DriveTrain driveTrain, FieldSim fieldSim) {
        m_driveTrain = driveTrain;
        m_fieldSim = fieldSim;

        for (int j = 0; j < endPointsRaw.length; j++) {
            endPoints[j] = new Pose2d(Units.inchesToMeters(endPointsRaw[j][0]), Units.inchesToMeters(endPointsRaw[j][1]), new Rotation2d(Units.degreesToRadians(endPointsRaw[j][2])));
        }

        configA.setReversed(false);
        //configA.setEndVelocity(configA.getMaxVelocity());
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));
        configA.addConstraint(new CentripetalAccelerationConstraint(2));
        }

        @Override
        public void initialize() {
                currentPose = m_driveTrain.getRobotPose();
                trajectory = TrajectoryGenerator.generateTrajectory(currentPose,
                    List.of(),
                    endPoints[index],
                    configA);
                command = TrajectoryUtils.generateRamseteCommand(m_driveTrain, trajectory);
                command.initialize();
        }

        @Override
        public void execute() {
                command.execute();
                if (command.isFinished()) {
                        getNewTrajectory();
                }
        }

        private void getNewTrajectory() {
            index++;
            if (index == endPoints.length) {
                complete = true;
                return;
            }

            if (index == endPoints.length - 1)
                configA.setEndVelocity(0);
            currentPose = m_driveTrain.getRobotPose();
            trajectory = TrajectoryGenerator.generateTrajectory(currentPose,
                List.of(),
                endPoints[index],
                configA);
            command = TrajectoryUtils.generateRamseteCommand(m_driveTrain, trajectory);
            command.initialize();
        }

        @Override
        public void end(boolean interrupted) {

        }

        @Override
        public boolean isFinished() {
                return complete;
        }
    }

