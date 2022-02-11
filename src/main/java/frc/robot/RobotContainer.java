// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimbRoutine;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.LookToTarget;
import frc.robot.commands.TestClimb;
import frc.robot.commands.ToggleLimelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final static Drivetrain drivetrain = new Drivetrain();
  public final static Camera camera = new Camera();
  public final static Climb climb = new Climb();



  public final static DriveArcade driveArcade = new DriveArcade(drivetrain);
  public final static ToggleLimelight toggleLimelight = new ToggleLimelight(camera);
  public final static LookToTarget lookToTarget = new LookToTarget(drivetrain, camera);
  public final static ClimbRoutine climbRoutine = new ClimbRoutine(climb);

  public static Joystick stick = new Joystick(0);
  private final Button toggleLimelightButton = new JoystickButton(stick, Constants.toggleLimelightButtonID);
  private final Button lookToTargetButton = new JoystickButton(stick, Constants.lookToTargetButtonID);
  private final Button liftState = new JoystickButton(stick, 7);
  private final Button climbRoutineButton = new JoystickButton(stick, 8);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html#perpetualcommand do this instead
    drivetrain.setDefaultCommand(driveArcade);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    toggleLimelightButton.whenPressed(new ToggleLimelight(camera));
    lookToTargetButton.whileHeld(new LookToTarget(drivetrain, camera));
    liftState.toggleWhenPressed(new TestClimb(climb));
    climbRoutineButton.whileHeld(new ClimbRoutine(climb));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            Constants.ks,
            Constants.kv,
            Constants.ka
          ),
          Constants.kDriveKinematics,
          Constants.autoMaxVoltage
        );
      
    TrajectoryConfig config = 
      new TrajectoryConfig(
        Constants.kMaxSpeed,
        Constants.kMaxAcceleration
        )
        .setKinematics(Constants.kDriveKinematics)
        .addConstraint(autoVoltageConstraint);
  
    
    Trajectory trajectory;
    
    trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),

      
      List.of(
        new Translation2d(1/3.281, -1/3.281),
        new Translation2d(2/3.281, 1/3.281)
      ),

      //drives 3 feet forward
      new Pose2d(3/3.281, 0, new Rotation2d(Math.toRadians(0))),


      //pass config to trajectory
      config
    );

    //trajectory = Robot.getPathweaverTrajectory();

    var leftController = new PIDController(Constants.kp, 0, 0);
    var rightController = new PIDController(Constants.kp, 0, 0);
    

    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory,  
      drivetrain::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(
        Constants.ks,
        Constants.kv,
        Constants.ka
      ),
      Constants.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      leftController,
      rightController,
      (leftVolts, rightVolts) -> {
        drivetrain.tankDrive(leftVolts, rightVolts);

        SmartDashboard.putNumber("left measurement", drivetrain.leftEncoderSpeed());
        SmartDashboard.putNumber("left reference", leftController.getSetpoint());

        SmartDashboard.putNumber("right measurement", drivetrain.rightEncoderSpeed());
        SmartDashboard.putNumber("right reference", rightController.getSetpoint());
      },
      drivetrain
    );

    drivetrain.resetGyro(); //robot now assumes degree angle is 0
    drivetrain.resetOdometry(trajectory.getInitialPose());

    return ramseteCommand.andThen(() -> drivetrain.tankDrive(0, 0));
  }
}