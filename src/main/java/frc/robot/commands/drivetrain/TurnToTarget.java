// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class TurnToTarget extends CommandBase {
  /** Creates a new TurnToTarget. */

  PIDController controller = new PIDController(0.06, 0, 0);
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ksAuto, Constants.kvAuto, Constants.kaAuto);
  public TurnToTarget(Drivetrain drivetrain, Vision visionSubsystem) {
    controller.setTolerance(0.5);
    addRequirements(drivetrain);
    addRequirements(visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Drivetrain.tankDrive(
    -controller.calculate(Vision.getLimelightX(), 0),
    controller.calculate(Vision.getLimelightX(), 0)
    );
    /*
    Drivetrain.tankDrive(
    controller.calculate(Vision.getLimelightX(), 0)
    +feedforward.calculate(6, 0.5), //volts/second, and volts/second^2
    controller.calculate(Vision.getLimelightX(), 0)
    +feedforward.calculate(6, 0.5)
    );*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Drivetrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}