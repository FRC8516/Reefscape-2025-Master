// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.WristCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawWrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveWristToPosition extends Command {
    private final ClawWrist m_ClawWrist;
    private final String m_NewPosition;
    private Timer m_timer;
    boolean m_isdone = false;

  /** Creates a new MoveWristToPosition. */
  public MoveWristToPosition(ClawWrist m_PositionClawWrist, String whichPosition) {
    // Set local variables
    m_ClawWrist = m_PositionClawWrist;
    m_NewPosition = whichPosition;
    m_isdone = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ClawWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    m_isdone = false;
     //Start timer as second kill out this command
     m_timer = new Timer();
     m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_ClawWrist.MoveClawToPosition(m_NewPosition);
     /* This ensures this commands ends if sensor not found 
    if (m_timer.get() > Constants.ManipulatorConstants.kIntakeWraistTime) {
      m_isdone = true;
    }
   This checks to see if arm is position 
    if (m_ClawWrist.isClawWraistInPosition() == true) {
      m_isdone = true;
    }
      */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
