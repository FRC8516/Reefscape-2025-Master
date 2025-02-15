// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClawPositions;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.Commands.ElevatorCommands.MoveElevatorToPosition;
import frc.robot.Commands.WristCommands.MoveWristToPosition;
import frc.robot.subsystems.ClawWrist;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralScoringPositions extends SequentialCommandGroup {
  /** Creates a new CoralScoringPositions. */
  public CoralScoringPositions(ClawWrist mClawWrist, Elevator mElevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveWristToPosition(mClawWrist, ClawPositions.TransferPosition),
      new MoveElevatorToPosition(mElevator, ElevatorPositions.L1Position)
    );
  }


}
