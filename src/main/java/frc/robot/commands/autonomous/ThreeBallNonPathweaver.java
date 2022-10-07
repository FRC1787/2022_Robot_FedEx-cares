// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.AngleTurn;
import frc.robot.commands.drivetrain.DriveForward;
import frc.robot.commands.drivetrain.TurnToTarget;
import frc.robot.commands.intake.IntakeBalls;
import frc.robot.commands.shooter.ShootBalls;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallNonPathweaver extends SequentialCommandGroup {
  /** Creates a new ThreeBallNonPathweaver. */
  public ThreeBallNonPathweaver(Drivetrain drivetrain, Intake intake, Shooter shooter, Vision vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new ParallelCommandGroup(
        new DriveForward(drivetrain).withTimeout(0.8),
        new IntakeBalls(intake)
      ).withTimeout(1.5),
      new AngleTurn(drivetrain, -135),
      new WaitCommand(0.5),
      new ParallelCommandGroup(
        new ShootBalls(shooter, intake),
        new TurnToTarget(drivetrain)
      ).withTimeout(2.5),
      new AngleTurn(drivetrain, -30),
      new ParallelCommandGroup(
        new DriveForward(drivetrain),
        new IntakeBalls(intake)
      ).withTimeout(2.5),
      new AngleTurn(drivetrain, 85),
      // new DriveForward(drivetrain).withTimeout(1),
      new ParallelCommandGroup(
        new ShootBalls(shooter, intake),
        new TurnToTarget(drivetrain)
      ).withTimeout(2.5)
      
      

    );
  }
}
