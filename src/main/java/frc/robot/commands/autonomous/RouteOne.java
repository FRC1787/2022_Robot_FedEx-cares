// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.IntakeBalls;
import frc.robot.commands.shooter.ShootBalls;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RouteOne extends SequentialCommandGroup {
  /** Creates a new RouteOne. */
  public RouteOne(Drivetrain drivetrain, Intake intake, Shooter shooter, Vision vision) {
    Trajectory trajectory1 = Robot.loadTrajectoryFromFile("Route1Initial");
    Trajectory trajectory2 = Robot.loadTrajectoryFromFile("Route1Cycle");

    addCommands(
      RobotContainer.createCommandForTrajectory(trajectory1).withTimeout(3).withName("Trajectory1"),
      new IntakeBalls(intake).withTimeout(3),
      RobotContainer.createCommandForTrajectory(trajectory2).withTimeout(2).withName("Trajectory2"),
      new ShootBalls(shooter, intake).withTimeout(5)
    );
  }
}