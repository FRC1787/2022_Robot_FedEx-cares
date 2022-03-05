package frc.robot.commands.autonomous;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestRoutine extends SequentialCommandGroup {
  /** Creates a new TestRoutine. */
  public TestRoutine(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Trajectory trajectory1 = RobotContainer.loadTrajectoryFromFile("Test1");
    
    // addCommands(
    //   new InstantCommand(() -> {
    //     drivetrain.resetOdometry(trajectory1.getInitialPose());
    //   }),
    //   RobotContainer.createCommandForTrajectory(trajectory1, false).withTimeout(50).withName("Test1")
    //   // RobotContainer
    // );
  }
}