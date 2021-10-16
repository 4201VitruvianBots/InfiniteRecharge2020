package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.autonomous.SmartdashboardCommand;
import frc.robot.commands.autonomous.TurnInPlace;
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

public class GalacticSearch extends SequentialCommandGroup {
    public GalacticSearch(DriveTrain driveTrain, FieldSim fieldSim) {
        int[][] waypointsRaw = {
                {46,90,180},
                {90,120,180},
                {90,90,0},
                {150,60,135},
                {180,26,180},
                {180,60,-30},
                {180,150,180},
                {210,120,160},
                {240,110,150},
                {270,84,130},
                {300,52,180},
                {375,52,180}
        };
        Pose2d[] waypoints = new Pose2d[waypointsRaw.length];
        for (int j = 0; j < waypointsRaw.length; j++) {
                waypoints[j] = new Pose2d(Units.inchesToMeters(waypointsRaw[j][0]), Units.inchesToMeters(waypointsRaw[j][1]), new Rotation2d(Units.degreesToRadians(waypointsRaw[j][2])));
        }
        Pose2d startPosition = waypoints[0];


        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(10), Units.feetToMeters(4));
        configA.setReversed(true);
        //configA.setEndVelocity(configA.getMaxVelocity());
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));
        configA.addConstraint(new CentripetalAccelerationConstraint(1.7));

        addCommands(new SetDriveShifters(driveTrain, Constants.DriveConstants.inSlowGear),
                new SetOdometry(driveTrain, fieldSim, startPosition),
                new SetDriveNeutralMode(driveTrain, 0));

        double[] startVelocities = {
                        0,
                        configA.getMaxVelocity()/3,
                        configA.getMaxVelocity()/3,
                        configA.getMaxVelocity()/3,
                        configA.getMaxVelocity()/3,
                        configA.getMaxVelocity()/3,
                        configA.getMaxVelocity()/3,
                        configA.getMaxVelocity()/3,
                        configA.getMaxVelocity()/3,
                        configA.getMaxVelocity()/3,
                        configA.getMaxVelocity()/3};
        double[] endVelocities = {
                        configA.getMaxVelocity()/3,
                        configA.getMaxVelocity()/3,
                        configA.getMaxVelocity()/3,
                        configA.getMaxVelocity()/3,
                        configA.getMaxVelocity()/3,
                        configA.getMaxVelocity()/3,
                        configA.getMaxVelocity()/3,
                        configA.getMaxVelocity()/3,
                        configA.getMaxVelocity()/3,
                        configA.getMaxVelocity()/3,
                        configA.getMaxVelocity()/2};

        for(int i = 0; i < waypoints.length - 1; i++) {
                configA.setStartVelocity(startVelocities[i]);
                configA.setEndVelocity(endVelocities[i]);
                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints[i],
                List.of(),
                waypoints[i + 1],
                configA);
            var command = TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory);
            addCommands(command);
        }
    }
}

