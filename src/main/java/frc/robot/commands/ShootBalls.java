// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootBalls extends CommandBase {
  /** Creates a new ShootBalls. */
  public ShootBalls(Shooter shootSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Shooter.accelerator.get() != 0.52 || Shooter.backspinner.get() != 0.52) {
      Shooter.accelerateShooter(0.52, 0.52);
    }
    else {
      Shooter.indexer.set(0.2); //adjust this
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.accelerator.set(0);
    Shooter.indexer.set(0);
    Shooter.backspinner.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
