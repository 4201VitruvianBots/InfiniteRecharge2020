package frc.robot.simulation;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class FieldSim {
    private Field2d m_field2d;
    private final DriveTrain m_driveTrain;
    private final Turret m_turret;
    private final Shooter m_shooter;
    private final Powercell[] m_powercells = new Powercell[6];

    private int ballCount;
    private Pose2d[] intakePose = {
        new Pose2d(),
        new Pose2d(),
        new Pose2d(),
        new Pose2d()
    };

    public FieldSim(DriveTrain driveTrain, Turret turret, Shooter shooter) {
        m_driveTrain = driveTrain;
        m_turret = turret;
        m_shooter = shooter;

        for(int i = 0; i < 6; i++)
            m_powercells[i] = new Powercell("PowerCell_" + i);

        m_field2d = new Field2d();
    }

    public void initSim() {
        // Load 3 powercells into the robot
        for(int i = 0; i < 3; i++)
            m_powercells[i].setBallState(1);

        ballCount = 3;

        // Put 3 powercells in the trench;
        m_powercells[3].setBallPose(SimConstants.blueTrenchBallPos[0]);
        m_powercells[4].setBallPose(SimConstants.blueTrenchBallPos[1]);
        m_powercells[5].setBallPose(SimConstants.blueTrenchBallPos[2]);
    }

    private void updateIntakePoses() {
        /* Intake Points:
           ^: Front of the robot
              -------
             |   ^   |
             |       |
             0-------1
             |       |
             3-------2
         */

        // Look up rotating a point about another point in 2D space for the math explanation
        Pose2d robotPose = m_driveTrain.getRobotPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double cos = robotPose.getRotation().getCos();
        double sin = robotPose.getRotation().getSin();

        double deltaXa = (robotX - SimConstants.robotLength / 2.0 + SimConstants.intakeLength) - robotX;
        double deltaXb = (robotX - SimConstants.robotLength / 2.0) - robotX;
        double deltaYa = (robotY + SimConstants.robotWidth / 2.0 ) - robotY;
        double deltaYb = (robotY - SimConstants.robotWidth / 2.0 ) - robotY;

        intakePose[0] = new Pose2d(cos * deltaXa - sin * deltaYa + robotX,
                                   sin * deltaXa + cos * deltaYa + robotY,
                                      new Rotation2d());
        intakePose[1] = new Pose2d(cos * deltaXa - sin * deltaYb + robotX,
                                   sin * deltaXa + cos * deltaYb + robotY,
                                      new Rotation2d());
        intakePose[2] = new Pose2d(cos * deltaXb - sin * deltaYb + robotX,
                                   sin * deltaXb + cos * deltaYb + robotY,
                                      new Rotation2d());
        intakePose[3] = new Pose2d(cos * deltaXb - sin * deltaYa + robotX,
                                   sin * deltaXb + cos * deltaYa + robotY,
                                      new Rotation2d());
    }

    private boolean isBallInIntakeZone(Pose2d ballPose){
        // The rise/run between intake points 0 to 1
        // Since the intake is a rectangle, this is the same as the slope between points 2 to 3
        double slope0to1 = (intakePose[1].getY() - intakePose[0].getY()) /(intakePose[1].getX() - intakePose[0].getX());

        // The rise/run between points 1 to 2
        // Same as slope between points 3 and 0
        double slope1to2 = (intakePose[2].getY() - intakePose[1].getY()) /(intakePose[2].getX() - intakePose[1].getX());

        // Use point-slope form to check if ball pose is above or below each line on the intake rectangle
        // For each pair of parallel lines, the ball needs to be above one line and below the other
        // Note: it's very important that the points be in the same order as the diagram above
        return (
                (ballPose.getY() >= slope0to1 * (ballPose.getX() - intakePose[0].getX()) + intakePose[0].getY()) ==
                        (ballPose.getY() <= slope0to1 * (ballPose.getX() - intakePose[2].getX()) + intakePose[2].getY())
            ) && (
                (ballPose.getY() >= slope1to2 * (ballPose.getX() - intakePose[0].getX()) + intakePose[0].getY()) ==
                        (ballPose.getY() <= slope1to2 * (ballPose.getX() - intakePose[1].getX()) + intakePose[1].getY())
        );

        /*List<Double> xValues = new ArrayList<>();
        List<Double> yValues = new ArrayList<>();

        for (Pose2d p:intakePose) {
            xValues.add(p.getX());
            yValues.add(p.getY());
        }

        // This is technically cheating, need a more accurate comparison
        double minX = Collections.min(xValues, null);
        double maxX = Collections.max(xValues, null);
        double minY = Collections.min(yValues, null);
        double maxY = Collections.max(yValues, null);

        if (maxX > ballPose.getX() && ballPose.getX() > minX &&
            maxY > ballPose.getY() && ballPose.getY() > minY)
            return true;
        else
            return false;*/
    }

    public void simulationPeriodic() {

        m_field2d.setRobotPose(m_driveTrain.getRobotPose());
        m_field2d.getObject("Turret").setPose(new Pose2d(m_driveTrain.getRobotPose().getTranslation(),
                                                                new Rotation2d(Math.toRadians(getIdealTurretAngle()))));

        updateIntakePoses();

//        m_field2d.getObject("Intake A").setPose(intakePose[0]);
//        m_field2d.getObject("Intake B").setPose(intakePose[1]);
//        m_field2d.getObject("Intake C").setPose(intakePose[2]);
//        m_field2d.getObject("Intake D").setPose(intakePose[3]);

        for(Powercell p:m_powercells) {
            updateBallState(p);
            m_field2d.getObject(p.getName()).setPose(p.getBallPose());
        }

        SmartDashboard.putData("Field2d", m_field2d);
    }

    public double getIdealTargetDistance() {
        return Math.sqrt(Math.pow(SimConstants.blueGoalPose.getY() - m_turret.getTurretSimPose().getY(), 2) + Math.pow(SimConstants.blueGoalPose.getX() - m_turret.getTurretSimPose().getX(), 2));
    }

    public double getIdealTurretAngle() {

        double targetRadians = Math.atan2(SimConstants.blueGoalPose.getY() -m_turret.getTurretSimPose().getY(), SimConstants.blueGoalPose.getX() - m_turret.getTurretSimPose().getX());

        return Math.toDegrees(targetRadians);
    }

    public Powercell[] getPowerCells() {
        return m_powercells;
    }

    public Pose2d getFieldSiMRobotPose() {
        return m_field2d.getRobotPose();
    }

    private void updateBallState(Powercell powercell) {
        Pose2d ballPose = powercell.getBallPose();

        if(ballPose.getX() < 0 || ballPose.getX() > SimConstants.fieldWidth || ballPose.getY() < 0 || ballPose.getY() > SimConstants.fieldHieght)
            powercell.setBallState(3);

//        System.out.println("Ball Shot: " + wasShot + "\tBall State: " + ballState);
//        System.out.println("Ball State: " + ballState + "\tCos: " + ballPose.getRotation().getCos() + "\tX Pos: " + ballPose.getX());
        switch (powercell.getBallState()) {
            case 3:
                // Ball is out of bounds
                if(ballPose.getX() < SimConstants.fieldWidth / 2.0)
                    powercell.setBallPose(SimConstants.redLoadingStation);
                else
                    powercell.setBallPose(SimConstants.blueLoadingStation);

                powercell.setBallState(0);
                break;
            case 2:
                // Ball is traveling in the air
                double currentTime = RobotController.getFPGATime();
                // FPGA time is in microseonds, need to convert it into seconds
                double deltaT = (currentTime - powercell.getLastTimestamp()) / 1e6;
                double distanceTraveled = SimConstants.shotSpeed * deltaT;
                double deltaX = distanceTraveled * ballPose.getRotation().getCos();
                double deltaY = distanceTraveled * ballPose.getRotation().getSin();
//                System.out.println("Delta X: " + deltaX + "\tDelta Y: " + deltaY + "\tDelta T: " + deltaT);
                powercell.setBallPose(new Pose2d(deltaX + ballPose.getX(),
                                                 deltaY + ballPose.getY(),
                                                     ballPose.getRotation()));

                powercell.setLastTimestamp(currentTime);
                break;
            case 1:
                // Ball has been picked up by the robot
                powercell.setBallPose(m_field2d.getObject("Turret").getPose());

                // Ball has been shot;
                if(powercell.getBallShotState()) {
                    powercell.setBallShotState(false);
                    powercell.setLastTimestamp(RobotController.getFPGATime());
                    powercell.setBallState(2);
                    ballCount--;
                }
                break;
            case 0:
            default:
                if(isBallInIntakeZone(ballPose)) {
                    ballCount++;
                    powercell.setBallState(1);
                }
                break;
        }
    }
}
