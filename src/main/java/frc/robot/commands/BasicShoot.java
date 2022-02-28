// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Shooter;

public class BasicShoot extends CommandBase {
  /** Creates a new BasicShoot. */
  double indexerSpeed, flywheelSpeed, backspinnerSpeed;
  public BasicShoot(Shooter shootSubsystem, double indexerSpeed, double flywheelSpeed, double backspinnerSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootSubsystem);
    this.indexerSpeed=indexerSpeed;
    this.flywheelSpeed=flywheelSpeed;
    this.backspinnerSpeed=backspinnerSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Shooter.setIndexerSpeed(indexerSpeed);
    Shooter.setAcceleratorSpeed(flywheelSpeed);
    Shooter.setBackspinnerSpeed(backspinnerSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.setIndexerSpeed(0);
    Shooter.setAcceleratorSpeed(0);
    Shooter.setBackspinnerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
