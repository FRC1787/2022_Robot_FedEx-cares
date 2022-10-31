// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeBalls;
import frc.robot.commands.intake.TimedIntakeBalls;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootBallsWithColor extends SequentialCommandGroup {

  /** Creates a new ShootBallsWithColor. */
  public ShootBallsWithColor(Shooter shooter, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TimedIntakeBalls(intake, 0.4, 1.0),
      new WheelToRPM(shooter, intake),
      new TimedIndexBalls(shooter, 1.0, 500)
    );
  }
}
