// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShootBalls extends CommandBase {
  /** Creates a new ShootBalls. */
  double acceleratorSpeed, backspinnerSpeed, indexerSpeed;
  public ShootBalls(Shooter shootSubsystem, double acceleratorSpeed, double backspinnerSpeed, double indexerSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootSubsystem);
    this.acceleratorSpeed=acceleratorSpeed;
    this.backspinnerSpeed=backspinnerSpeed;
    this.indexerSpeed=indexerSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Shooter.getAcceleratorSpeed() != acceleratorSpeed || Shooter.getBackspinnerSpeed() != backspinnerSpeed) {
      Shooter.accelerateShooter(acceleratorSpeed, backspinnerSpeed);
    }
    else {
      Shooter.setIndexerSpeed(indexerSpeed); //adjust this
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.setAcceleratorSpeed(0);
    Shooter.setIndexerSpeed(0);
    Shooter.setBackspinnerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
