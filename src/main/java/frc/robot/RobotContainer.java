// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.climb.ClimbRoutine;
import frc.robot.commands.climb.MoveArm;
import frc.robot.commands.climb.TestClimb;
import frc.robot.commands.drivetrain.AngleTurn;
import frc.robot.commands.drivetrain.DriveArcade;
import frc.robot.commands.drivetrain.TurnToTarget;
import frc.robot.commands.intake.IntakeBalls;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.shooter.AllShot;
import frc.robot.commands.shooter.ShootBalls;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Climb;
import frc.robot.commands.autonomous.NonPathweaver;
import frc.robot.commands.autonomous.ThreeBallNonPathweaver;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();


  // SUBSYSTEMS
  public final static Vision     vision     = new Vision();
  public final static Climb      climb      = new Climb();
  public final static Drivetrain drivetrain = new Drivetrain();
  public final static Intake     intake     = new Intake();
  public final static Shooter    shooter    = new Shooter();


  // COMMANDS
      public final static DriveArcade   driveArcade = new DriveArcade(drivetrain);
  public final static Command autoCommand = new ThreeBallNonPathweaver(drivetrain, intake, shooter, vision);
    //  public final static Command autoCommand = new NonPathweaver(drivetrain, intake, shooter, vision);
  // Controllers
    public static Joystick stick = new Joystick(0);
    // public static XboxController stick = new XboxController(0);

  // Button Bindings
    // Camera
      //private final Button toggleLimelightButton = new JoystickButton(stick, Constants.toggleLimelightButtonID);
      private final Button turnToTargetButton = new JoystickButton(stick, Constants.turnToTargetButtonID);
    // Climb
      private final Button manualMoveArmButton = new JoystickButton(stick, Constants.manualMoveArmButtonID);
      private final Button climbRoutineButton = new JoystickButton(stick, Constants.climbRoutineButtonID);
      private final Button fullExtendButton = new JoystickButton(stick, Constants.fullExtendButtonID);
      private final Button fullRetractButton = new JoystickButton(stick, Constants.fullRetractButtonID);
      private final Button toggleClimbButton = new JoystickButton(stick, Constants.manualArmPneumaticButtonID);
    
      // Drivetrain

    // Intake
      private final Button intakeBallsButton = new JoystickButton(stick, Constants.intakeBallsButtonID);
      private final Button reverseIntakeButton = new JoystickButton(stick, Constants.reverseIntakeButtonID);
    // Shooter
      private final Button farShootButton = new JoystickButton(stick, Constants.farShootButtonID);
      private final Button closeShootButton = new JoystickButton(stick, Constants.closeShootButtonID);
      private final Button shooterToggle = new JoystickButton(stick, Constants.shooterToggleButtonID);
      private final Button testShootButton = new JoystickButton(stick, 4);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html#perpetualcommand do this instead
    drivetrain.setDefaultCommand(driveArcade);

    // Adds autonomous routine options :D
    autoChooser.setDefaultOption("Non-Pathweaver", new NonPathweaver(drivetrain, intake, shooter, vision));
    //autoChooser.addOption("Three Ball Auto", new ThreeBallNonPathweaver(drivetrain, intake, shooter, vision));
    // Sends the routine options to SmartDashboard
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //toggleLimelightButton.whenPressed(new ToggleLimelight(vision));
    turnToTargetButton.whenActive(new TurnToTarget(drivetrain));

    
    manualMoveArmButton.toggleWhenPressed(new TestClimb(climb));
    climbRoutineButton.whileHeld(new ClimbRoutine(climb));
    fullExtendButton.whenPressed(new MoveArm(climb, .8));
    fullRetractButton.whenPressed(new MoveArm(climb, -.8));
    toggleClimbButton.whenPressed(new InstantCommand(Climb::toggleClimbSolenoid, climb));

    intakeBallsButton.whileHeld(new IntakeBalls(intake));
    reverseIntakeButton.whileHeld(new ReverseIntake(intake, shooter));

    shooterToggle.whenPressed(new InstantCommand(Shooter::toggleShooterPosition, shooter));
    closeShootButton.whenHeld(
      new InstantCommand(Shooter::lowerShooter, shooter)
      .andThen(
        new ShootBalls(shooter, intake)));
    farShootButton.whenHeld(
      new InstantCommand(Shooter::raiseShooter, shooter)
      .andThen(
        new ParallelCommandGroup(
          new ShootBalls(shooter, intake),
          new TurnToTarget(drivetrain))));
    
    testShootButton.whenHeld(
      new InstantCommand(Shooter::raiseShooter, shooter)
      .andThen(
        new ParallelCommandGroup(
          new TurnToTarget(drivetrain),
          new AllShot(shooter, intake)
        )
      ));
  }

    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoCommand;
    //return autoChooser.getSelected();
  }

  /**
   * Creates a command that controls the drivetrain to follow a given trajectory during autonomous
   * 
   * @param trajectory - The trajectory to follow
   * 
   * @return command that will have the robot follow the given trajectory
   */
  public static Command createCommandForTrajectory(Trajectory trajectory) {
    // drivetrain.resetEncoders();
    drivetrain.resetOdometry(trajectory.getInitialPose());

    var leftController = new PIDController(Constants.kpAuto, 0, 0);
    var rightController = new PIDController(Constants.kpAuto, 0, 0);

    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory,  
      drivetrain::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(
          Constants.ksAuto,
          Constants.kvAuto,
          Constants.kaAuto
        ),
      Constants.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      leftController,
      rightController,
      (leftVolts, rightVolts) -> {
        Drivetrain.tankDrive(leftVolts, rightVolts);

        SmartDashboard.putNumber("left measurement", Drivetrain.leftEncoderSpeed());
        SmartDashboard.putNumber("left reference", leftController.getSetpoint());

        SmartDashboard.putNumber("right measurement", Drivetrain.rightEncoderSpeed());
        SmartDashboard.putNumber("right reference", rightController.getSetpoint());
      },
      drivetrain
    );
    return ramseteCommand.andThen(() -> Drivetrain.tankDrive(0, 0));
  }



}