// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestRoutine extends SequentialCommandGroup {
  /** Creates a new TestRoutine. */
  public TestRoutine(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Trajectory trajectory1 = drivetrain.loadTrajectoryFromFile("Test1");
    
    addCommands(
      new InstantCommand(() -> {
        drivetrain.resetOdometry(trajectory1.getInitialPose());
      }),
      // drivetrain.createCommandForTrajectory(trajectory1, false).withTimeout(50).withName("Bounce1"),
      
    );
  }
}
