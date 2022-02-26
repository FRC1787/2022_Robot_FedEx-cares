// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climb;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbRoutine extends SequentialCommandGroup {
  /** Creates a new ClimbRoutine. */
  public ClimbRoutine(Climb climb) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      //DONT put these values in constants it is easier to do them here anyways

      //assumes driver starts with arms attached to first bar and arms extended
      new MoveArm(climb, -0.6), //pull robot up by retracting arms
      new WaitCommand(0.25),
      new PartialMoveArm(climb, 0.7, 0.3), //extend arm to clear bar
      new WaitCommand(0.25),
      new SetClimbPiston(climb, false), //lean arm back
      new WaitCommand(0.25),
      new MoveArm(climb, 0.7), //extend arm all the way
      new WaitCommand(0.25),
      new SetClimbPiston(climb, true), //push arm up against bar
      new WaitCommand(0.75),
      new MoveArm(climb, -0.6), //retract arm to pull robot to second bar
      new WaitCommand(1.0) //wait time to account for swinging (can be lowered)

    );
  }
}