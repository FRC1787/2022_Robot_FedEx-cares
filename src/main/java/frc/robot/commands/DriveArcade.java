// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveArcade extends CommandBase {
  /** Creates a new DriveArcade. */
  public DriveArcade(Drivetrain ggug) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ggug);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  public double deadzone(double num) {
    return Math.abs(num) > 0.025 ? num : 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double linearSpeed = deadzone(RobotContainer.stick.getY());
    double angularSpeed = deadzone(RobotContainer.stick.getX());

    Drivetrain.moveLeftSide(linearSpeed - angularSpeed);
    Drivetrain.moveRightSide(linearSpeed + angularSpeed);
    SmartDashboard.putNumber("left distance", Drivetrain.leftDistance());
    SmartDashboard.putNumber("right distance", Drivetrain.rightDistance());
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
