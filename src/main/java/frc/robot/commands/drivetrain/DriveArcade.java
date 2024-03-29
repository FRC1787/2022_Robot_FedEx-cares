// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveArcade extends CommandBase {
  /** Creates a new DriveArcade. */
  public DriveArcade(Drivetrain drivetrain) {
    // Use addRequirements() here to declare sub   dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private double deadzone(double num) {
    return Math.abs(num) > 0.02 ? num : 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double stickY = RobotContainer.stick.getY();
    double stickX = RobotContainer.stick.getX();
    // double brickY = RobotContainer.brick.getLeftY();
    // double brickX = RobotContainer.brick.getLeftX();

    double linearSpeed = Math.signum(stickY)*Math.pow(deadzone(stickY), 2) + Math.pow(deadzone(stickY), 3);
    double angularSpeed = Math.signum(stickX)*Math.pow(deadzone(stickX), 2) + Math.pow(deadzone(stickX), 3);

    Drivetrain.moveLeftSide(-linearSpeed + angularSpeed);
    Drivetrain.moveRightSide(-linearSpeed - angularSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
