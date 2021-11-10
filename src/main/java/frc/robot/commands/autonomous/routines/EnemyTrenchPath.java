package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.SetDriveNeutralMode;
import frc.robot.commands.drivetrain.SetOdometry;
import frc.robot.commands.intake.AutoControlledIntake;
import frc.robot.commands.intake.SetIntakePiston;
import frc.robot.commands.shooter.AutoRapidFireSetpoint;
import frc.robot.commands.shooter.SetAndHoldRpmSetpoint;
import frc.robot.commands.turret.AutoUseVisionCorrection;
import frc.robot.commands.turret.SetTurretRobotRelativeAngle;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.Arrays;

public class EnemyTrenchPath extends SequentialCommandGroup {
    public EnemyTrenchPath(DriveTrain driveTrain, Intake intake, Indexer indexer, Turret turret, Shooter shooter, Vision vision, FieldSim fieldSim) {
        Pose2d[] startToEnemyTrench = {
            new Pose2d(12.75,7.275692, new Rotation2d(Units.degreesToRadians(0))),
            new Pose2d(10.1,7.275692, new Rotation2d(Units.degreesToRadians(0)))
        };
        Pose2d[] enemyTrenchToShoot = {
                new Pose2d(10.1,7.275692, new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(11,6.275692, new Rotation2d(Units.degreesToRadians(-90))),
                new Pose2d(11,4, new Rotation2d(Units.degreesToRadians(-90))),
                new Pose2d(12,3, new Rotation2d(Units.degreesToRadians(0)))
        };

        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(6), Units.feetToMeters(10));
        configA.setReversed(true);
        configA.setEndVelocity(0);
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));

        var startToEnemyTrenchTrajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(startToEnemyTrench.clone()), configA);
        var startToEnemyTrenchCommand = TrajectoryUtils.generateRamseteCommand(driveTrain, startToEnemyTrenchTrajectory);

        var configB = new TrajectoryConfig(Units.feetToMeters(6), Units.feetToMeters(4));
        configB.setReversed(false);
        configB.setEndVelocity(0);
        configB.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configB.getMaxVelocity()));
        configB.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));
        configB.addConstraint(new CentripetalAccelerationConstraint(Units.feetToMeters(1.5)));

        var enemyTrenchToShootTrajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(enemyTrenchToShoot.clone()), configB);
        var enemyTrenchToShootCommand = TrajectoryUtils.generateRamseteCommand(driveTrain, enemyTrenchToShootTrajectory);

        if(RobotBase.isReal())
            addCommands(
                    new SetOdometry(driveTrain, fieldSim, startToEnemyTrench[0]),
                    new SetDriveNeutralMode(driveTrain,0),
                    new SetIntakePiston(intake, true),
                    new ParallelDeadlineGroup(
                            startToEnemyTrenchCommand,
                            new AutoControlledIntake(intake, indexer)
                    ).andThen(()->driveTrain.setMotorTankDrive(0,0)),
                    new SetIntakePiston(intake, false),
                    enemyTrenchToShootCommand,
                    new SetAndHoldRpmSetpoint(shooter, vision, 3800),
                    new SetTurretRobotRelativeAngle(turret, -75).withTimeout(0.5),
                    new AutoUseVisionCorrection(turret, vision).withTimeout(0.5),
                    new ConditionalCommand(new WaitCommand(0),
                                           new WaitCommand(0.5),
                                           shooter::canShoot),
                    new AutoRapidFireSetpoint(shooter, indexer, intake,1).withTimeout(1)
            );
        else
            addCommands(
                    new SetOdometry(driveTrain, fieldSim, startToEnemyTrench[0]),
                    startToEnemyTrenchCommand.andThen(()->driveTrain.setMotorTankDrive(0,0)),
                    new WaitCommand(1),
                    enemyTrenchToShootCommand.andThen(()->driveTrain.setMotorTankDrive(0,0))
            );
    }
}

