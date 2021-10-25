/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/**
 * An example command that uses an example subsystem.
 */
public class TestAutomatedShooting extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain m_driveTrain;
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final Vision m_vision;
    private double m_RPM;

    
    private final double shooterDistanceFromFront = Units.inchesToMeters(22.5);
    private final double metersPerSecondToRPM = 485;

    /**
     * Creates a new ExampleCommand.
     */
    public TestAutomatedShooting(DriveTrain driveTrain, Shooter shooter, Turret turret, Vision vision) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_driveTrain = driveTrain;
        m_shooter = shooter;
        m_turret = turret;
        m_vision = vision;
        m_RPM = 0;
        addRequirements(turret);
//  addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_turret.setControlMode(1);
        
        double distance = Units.feetToMeters(m_vision.getTargetDistance());
        SmartDashboardTab.putNumber("Shooter", "Distance", distance);
        m_RPM = (distance * Math.sqrt(0.5 * Constants.g / (distance * Math.tan(Constants.verticalShooterAngle) - Constants.verticalTargetDistance)) / Math.cos(Constants.verticalShooterAngle))
        * metersPerSecondToRPM;
        if (m_RPM > 5000) {
            m_RPM = 0;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_vision.setLastValidTargetTime();
        m_shooter.setRPM(m_RPM);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.setRPM(- 1);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
