// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class TurnToTarget extends CommandBase {
  /** Creates a new TurnToTarget. */

  PIDController controller = new PIDController(0.085, 0, 0.045);
  

  public TurnToTarget(Drivetrain drivetrain) {
    controller.setTolerance(0.75); //TODO: try lowering this and tuning pid
    addRequirements(drivetrain);
    // if (Robot.inAuto) {
    //   controller = new PIDController(0.10, 0, 0.02);
    // }
    // else {
      // controller = new PIDController(0.05, 0, 0.03);
    // }
    controller.setSetpoint(3);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Drivetrain.tankDrive(
    -controller.calculate(Vision.getLimelightX()),
    controller.calculate(Vision.getLimelightX())
    );
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
