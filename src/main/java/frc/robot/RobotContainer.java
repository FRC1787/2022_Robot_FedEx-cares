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
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.LookToTarget;
import frc.robot.commands.ToggleLimelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Camera;
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



  public final static DriveArcade driveArcade = new DriveArcade(drivetrain);
  public final static ToggleLimelight toggleLimelight = new ToggleLimelight(camera);
  public final static LookToTarget lookToTarget = new LookToTarget(drivetrain, camera);


  public static Joystick stick = new Joystick(0);
  private final Button toggleLimelightButton = new JoystickButton(stick, Constants.toggleLimelightButtonID);
  private final Button lookToTargetButton = new JoystickButton(stick, Constants.lookToTargetButtonID);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
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
    
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      //start at 0,0 with degree angle of 0
      new Pose2d(0, 0, new Rotation2d(0)),

      //hit 1, 1 and 2,-1 as interior waypoints
      List.of(
        new Translation2d(1, 1),
        new Translation2d(2, -1)
      ),

      //end at 3,0 facing 0 again
      new Pose2d(3, 0, new Rotation2d(0)),

      //pass config to trajectory
      config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory,
      drivetrain::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(
        Constants.ks,
        Constants.kv,
        Constants.ka
      ),
      Constants.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(Constants.kp, 0, 0),
      new PIDController(Constants.kp, 0, 0),
      drivetrain::tankDrive,
      drivetrain
    );

    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    return ramseteCommand.andThen(() -> drivetrain.tankDrive(0, 0));
  }
}
