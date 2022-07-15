// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class FollowTarget extends CommandBase {
  /** Creates a new FollowTarget. */

  private final PIDController linearController = new PIDController(1.1, 0, 0);
  private final PIDController angularController = new PIDController(0.002, 0, 0);

  private double linearSpeed;
  private double angularSpeed;

  public FollowTarget(Drivetrain drivetrain) {
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    linearController.setSetpoint(0.1);
    angularController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Vision.hasTargets()) {
      linearSpeed = -linearController.calculate(Vision.limelightDistance());
      angularSpeed = -angularController.calculate(Vision.getLimelightX());
    }
    else {
      linearSpeed = 0; angularSpeed = 0;
    }
    SmartDashboard.putNumber("linear speed", linearSpeed);
    SmartDashboard.putNumber("angular speed", angularSpeed);
    Drivetrain.moveLeftSide(linearSpeed + angularSpeed);
    Drivetrain.moveRightSide(linearSpeed - angularSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Drivetrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Vision.limelightArea() > 14;
  }
}
