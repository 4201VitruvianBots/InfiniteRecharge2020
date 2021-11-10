/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * An example command that uses an example subsystem.
 */
public class SetClimberOutput extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber m_climber;
    private final Joystick m_controller;

    private boolean currentDirection = true;
    private boolean movable, switchDirection;
    private double timestamp;
    private int lastDirection;

    /*
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SetClimberOutput(Climber climber, Joystick controller) {
        m_climber = climber;
        m_controller = controller;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double input = Math.abs(m_controller.getRawAxis(5)) > 0.2 ? m_controller.getRawAxis(5) : 0;
        
        int direction = input > 0 ? 1 : input < 0 ? - 1 : 0;
        // direction = (int) Math.signum(input);
        // might be better
        if(m_climber.getClimbState()) {
            SmartDashboardTab.putNumber("Climber", "Direction", direction);
            SmartDashboardTab.putBoolean("Climber", "currentDirection", currentDirection);

            if(direction != lastDirection) {
                timestamp = Timer.getFPGATimestamp();
                movable = false;
                if(direction == 1 ) {
                    switchDirection = true;
                } else if(direction <= 0) {
                    switchDirection = false;
                }
            }

            if(movable) {
                double output = input;
                m_climber.setClimberOutput(output);
            } else {
                if(switchDirection)
                    climberReleaseSequence();
                else {
                    m_climber.setClimbPiston(false);
                    movable = true;
                    currentDirection = true;
                    //climberRetractSequence();
                }
            }
            lastDirection = direction;
        }
    }

    private void climberReleaseSequence() {
        SmartDashboardTab.putString("Climber", "SetClimberOutput", "Releasing");
        m_climber.setClimbPiston(true);
        m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.4);
        m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.4);
        if(Math.abs(Timer.getFPGATimestamp() - timestamp) < 0.2)
            m_climber.setClimberOutput(- 0.35);
        else if(Math.abs(Timer.getFPGATimestamp() - timestamp) < 0.4)
            m_climber.setClimberOutput(0.25);
        else {
            m_climber.setClimberOutput(0);
            movable = true;
            currentDirection = true;
            m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        }
    }

    private void climberRetractSequence() {
        SmartDashboardTab.putString("Climber", "SetClimberOutput", "Retracting");
        m_climber.setClimbPiston(false);
        m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.4);
        m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.4);
        if(Math.abs(Timer.getFPGATimestamp() - timestamp) < 0.2)
            m_climber.setClimberOutput(- 0.25);
        else {
            m_climber.setClimberOutput(0);
            movable = true;
            currentDirection = false;
            m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_climber.setClimberOutput(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
