// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AngleTurn extends CommandBase {
  /** Creates a new AngleTurn. */

  double initAngle;
  double angle;

  PIDController controller = new PIDController(0, 0, 0);

  public AngleTurn(Drivetrain drivetrain, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.angle=angle;
    controller.setTolerance(5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initAngle = Drivetrain.getAngle();
    controller.setSetpoint(initAngle+angle);
    if (angle > 0) Drivetrain.tankDrive(3, -3);
    else Drivetrain.tankDrive(-3, 3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controller.calculate(Drivetrain.getAngle());
    SmartDashboard.putNumber("initAngle", initAngle);
    SmartDashboard.putNumber("targetAngle", initAngle+angle);
    SmartDashboard.putBoolean("atSetpoint", controller.atSetpoint());
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
