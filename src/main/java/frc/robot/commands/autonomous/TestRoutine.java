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
public class TestRoutine extends SequentialCommandGroup {
  /** Creates a new TestRoutine. */
  public TestRoutine(Drivetrain drivetrain, Intake intake, Shooter shooter, Vision vision) {
    Trajectory trajectory1 = Robot.loadTrajectoryFromFile("Test1");
    Trajectory trajectory2 = Robot.loadTrajectoryFromFile("Test2");

    Drivetrain.resetGyro();
    drivetrain.resetOdometry(trajectory1.getInitialPose());

    addCommands(
      // Drives to cargo on field
      RobotContainer.createCommandForTrajectory(trajectory1).withTimeout(8).withName("Test1"),
      // Intakes cargo for 2 seconds
      new IntakeBalls(intake).withTimeout(3),
      // Turns to face the hub
      RobotContainer.createCommandForTrajectory(trajectory2).withTimeout(8).withName("Test2"),
      // Shoots the cargo for 5 seconds to shoot both cargo balls
      new ShootBalls(shooter, vision, intake).withTimeout(5)
    );
  }
}