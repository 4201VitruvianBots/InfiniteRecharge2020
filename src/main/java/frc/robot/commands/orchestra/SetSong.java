/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TalonOrchestra;

public class SetSong extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TalonOrchestra m_orchestra;
  private String m_song;

  public SetSong(TalonOrchestra orchestra, String song) {
    m_orchestra = orchestra;
    m_song = song;
    addRequirements(orchestra);
  }

  @Override
  public void initialize() {
    m_orchestra.song(m_song);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    //stops the song
    m_orchestra.stopSong();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
