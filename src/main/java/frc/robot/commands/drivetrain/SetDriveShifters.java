/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/**
 * An example command that uses an example subsystem.
 */
public class SetDriveShifters extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain m_driveTrain;
    private final boolean m_shifterState;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SetDriveShifters(DriveTrain subsystem, boolean shifterState) {
        m_driveTrain = subsystem;
        m_shifterState = shifterState;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
//        if(RobotBase.isReal()) {
//            if (m_driveTrain.getDriveShifterStatus() != m_shifterState)
//                m_driveTrain.setDriveShifterStatus(m_shifterState);
//        } else
//            m_driveTrain.setDriveShifterStatus(m_shifterState);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
