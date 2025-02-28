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
  public CoralScoringPositions(ClawWrist mClawWrist, Elevator mElevator, String Position) {
    if (Position == "L1") {
      addCommands(
 //       new MoveWristToPosition(mClawWrist, ClawPositions.TransferPosition).withTimeout(0.25),
 new MoveWristToPosition(mClawWrist, ClawPositions.ScoringPosition).withTimeout(1.5),     
 new MoveElevatorToPosition(mElevator, ElevatorPositions.L1Position).withTimeout(1.5)
      
      );
    } else if (Position == "L2"){
      addCommands(
        new MoveWristToPosition(mClawWrist, ClawPositions.TransferPosition).withTimeout(1.5),
        new MoveElevatorToPosition(mElevator, ElevatorPositions.L2Position).withTimeout(1.5),
        new MoveWristToPosition(mClawWrist, ClawPositions.ScoringPosition).withTimeout(1.5)
      );
    } else if (Position == "L3"){
      addCommands(
        new MoveWristToPosition(mClawWrist, ClawPositions.TransferPosition).withTimeout(1.5),
        new MoveElevatorToPosition(mElevator, ElevatorPositions.L3Position).withTimeout(1.75),
        new MoveWristToPosition(mClawWrist, ClawPositions.ScoringPosition).withTimeout(1.5)
      );
    } else if (Position == "L4"){
      addCommands(
        new MoveWristToPosition(mClawWrist, ClawPositions.TransferPosition).withTimeout(1.5),
        new MoveElevatorToPosition(mElevator, ElevatorPositions.L4Position).withTimeout(1.5),
        new MoveWristToPosition(mClawWrist, ClawPositions.ScoringPosition).withTimeout(1.5)
      );
    } else if (Position == "Home"){
      addCommands(
        new MoveWristToPosition(mClawWrist, ClawPositions.TransferPosition).withTimeout(1.5),
        new MoveElevatorToPosition(mElevator, ElevatorPositions.HomePosition).withTimeout(1.75),
        new MoveWristToPosition(mClawWrist, ClawPositions.HomePosition).withTimeout(1.5)
      );
    } else if (Position == "clawHome"){
      addCommands(
        new MoveWristToPosition(mClawWrist, ClawPositions.HomePosition).withTimeout(1.5)
      );
    } else if (Position == "clawTransfer"){
      addCommands(
        new MoveWristToPosition(mClawWrist, ClawPositions.TransferPosition).withTimeout(1.5)
      );
    } else if (Position == "clawScoring"){
      addCommands(
        new MoveWristToPosition(mClawWrist, ClawPositions.AlgaeScoringPosition).withTimeout(1.5)
      );
    } else if (Position == "clawAlgae"){
      addCommands(
        new MoveWristToPosition(mClawWrist, ClawPositions.AlgaePosition).withTimeout(1.5)
      );
    } else if (Position == "ElevHome"){
      addCommands(
        new MoveElevatorToPosition(mElevator, ElevatorPositions.HomePosition)
      );
    } else if (Position == "ElevL2"){
      addCommands(
        new MoveElevatorToPosition(mElevator, ElevatorPositions.L2Position)
      );
    } else if (Position == "ElevL1"){
      addCommands(
        new MoveElevatorToPosition(mElevator, ElevatorPositions.L1Position)
      );
    } else if (Position == "ElevL3"){
      addCommands(
        new MoveElevatorToPosition(mElevator, ElevatorPositions.L3Position)
      );
    }
  }
}