// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveForward;
import frc.robot.commands.drivetrain.OneEighty;
import frc.robot.commands.drivetrain.TurnToTarget;
import frc.robot.commands.intake.IntakeBalls;
import frc.robot.commands.shooter.LowerShooter;
import frc.robot.commands.shooter.RaiseShooter;
import frc.robot.commands.shooter.ShootBalls;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NonPathweaver extends SequentialCommandGroup {
  /** Creates a new NonPathweaver. */
  public NonPathweaver(Drivetrain drivetrain, Intake intake, Shooter shooter, Vision vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Drivetrain.resetGyro();
    
    addCommands(
      new IntakeBalls(intake).withTimeout(0.5),
      
      new ParallelCommandGroup(
        new IntakeBalls(intake),
        new DriveForward(drivetrain) 
        ).withTimeout(2),
      new WaitCommand(.25),
      new OneEighty(drivetrain, vision).withTimeout(0.76531787),
      new DriveForward(drivetrain).withTimeout(1.5),
      new RaiseShooter(shooter),
      new TurnToTarget(drivetrain, vision),
      new ShootBalls(shooter, vision, intake).withTimeout(3)
    );
  }
}