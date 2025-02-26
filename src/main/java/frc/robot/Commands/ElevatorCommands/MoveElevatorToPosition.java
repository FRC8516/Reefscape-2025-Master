// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorToPosition extends Command {
  private final Elevator m_Elevator;
  private final String m_NewPosition;
  boolean m_isdone;
  /** Creates a new MoveElevatorToPosition. */
  public MoveElevatorToPosition(Elevator m_PositionElevator, String whichPosition) {
    // Set local variables
    m_Elevator = m_PositionElevator;
    m_NewPosition = whichPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isdone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Elevator.MoveElevatorToPosition(m_NewPosition);
    /* This checks to see if arm is position 
    if (m_Elevator.isElevatorInPosition() == true) {
      m_isdone = true;
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
