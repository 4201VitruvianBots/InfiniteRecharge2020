package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.ResetOdometry;
import frc.robot.commands.drivetrain.SetDriveNeutralMode;
import frc.robot.commands.drivetrain.SetDriveShifters;
import frc.robot.commands.drivetrain.SetOdometry;
import frc.robot.commands.intake.AutoControlledIntake;
import frc.robot.commands.intake.SetIntakePiston;
import frc.robot.commands.shooter.AutoRapidFireSetpoint;
import frc.robot.commands.shooter.AutoRapidFireSetpoint2;
import frc.robot.commands.shooter.SetAndHoldRpmSetpoint;
import frc.robot.commands.turret.AutoUseVisionCorrection;
import frc.robot.commands.turret.SetTurretRobotRelativeAngle;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.ArrayList;
import java.util.List;

public class AllyTrenchPathStraight extends SequentialCommandGroup {
    public AllyTrenchPathStraight(DriveTrain driveTrain, Intake intake, Indexer indexer, Turret turret, Shooter shooter, Vision vision) {
        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(6), Units.feetToMeters(4));
        configA.setReversed(true);
        configA.setEndVelocity(0);
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(), 10));

        //var startToTrenchPath = TrajectoryUtils.readCsvTrajectory("init1Ally2");
        ArrayList<Pose2d> startToTrenchPath = new ArrayList();
        startToTrenchPath.add(new Pose2d(0, 0, new Rotation2d(0)));
        startToTrenchPath.add(new Pose2d(-3.3, 0, new Rotation2d(0)));
        var startToTrenchCommand = TrajectoryUtils.generateRamseteCommand(driveTrain, startToTrenchPath, configA);

        var configB = new TrajectoryConfig(Units.feetToMeters(6), Units.feetToMeters(4));
        configB.setReversed(false);
        configB.setEndVelocity(0);
        configB.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configB.getMaxVelocity()));
        configB.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));
        ArrayList<Pose2d> trenchToShootPath = new ArrayList();
        trenchToShootPath.add(new Pose2d(0, 0, new Rotation2d(0)));
        trenchToShootPath.add(new Pose2d(3.3, 0, new Rotation2d(0)));

        var trenchToShootCommand = TrajectoryUtils.generateRamseteCommand(driveTrain, trenchToShootPath, configB);

        addCommands(
                new SetOdometry(driveTrain, new Pose2d()),
                new SetDriveNeutralMode(driveTrain, 0),
                new SetDriveShifters(driveTrain, false),
                new SetAndHoldRpmSetpoint(shooter, vision, 3800),
                new SetTurretRobotRelativeAngle(turret, - 25).withTimeout(0.25),
                new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
                new ConditionalCommand(new WaitCommand(0),
                        new WaitCommand(0.5),
                        shooter :: canShoot),
                new AutoRapidFireSetpoint(shooter, indexer, intake, 1).withTimeout(1.5),
                new SetIntakePiston(intake, true),
                new SetDriveShifters(driveTrain, false),
                new ParallelDeadlineGroup(
                        startToTrenchCommand,
                        new AutoControlledIntake(intake, indexer)
                ),
                new AutoControlledIntake(intake, indexer).withTimeout(0.25),
                new SetIntakePiston(intake, false),
                new SetOdometry(driveTrain, new Pose2d()),
                new ParallelDeadlineGroup(
                        trenchToShootCommand,
                        new SetTurretRobotRelativeAngle(turret, - 25).withTimeout(0.25),
                        new SetAndHoldRpmSetpoint(shooter, vision, 3800)
                ).andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
                new AutoUseVisionCorrection(turret, vision).withTimeout(0.75),
                new ConditionalCommand(new WaitCommand(0),
                        new WaitCommand(0.5),
                        shooter :: canShoot),
//                new ConditionalCommand(new AutoRapidFireSetpoint(shooter, indexer, intake,6),
//                                       new WaitCommand(0),
//                                       vision::hasTarget)
                new AutoRapidFireSetpoint2(shooter, indexer, intake, 6)
        );
    }
}
