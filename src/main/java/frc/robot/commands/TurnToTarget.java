// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToTarget extends PIDCommand {
  /** Creates a new TurnToTarget. */
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ksAuto, Constants.kvAuto, Constants.kaAuto);
  public TurnToTarget(Drivetrain drivetrainSubsystem, Vision cameraSubsystem) {
    super(
      // The controller that the command will use
      new PIDController(Constants.lookToTargetP, Constants.lookToTargetI, Constants.lookToTargetD),
      // This should return the measurement
      Vision::getLimelightX,
      // This should return the setpoint (can also be a constant)
      0,
      // This uses the output
      output -> {
        Drivetrain.moveLeftSide(-output);
        Drivetrain.moveRightSide(output);
      }
    );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drivetrainSubsystem);
    addRequirements(cameraSubsystem);
    //getController().enableContinuousInput(-180, 180);
    getController().setTolerance(0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
