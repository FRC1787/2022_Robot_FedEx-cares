// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class BasicShoot extends CommandBase {
  double indexerSpeed, flywheelSpeed, backspinnerSpeed;
  public BasicShoot(Shooter shootSubsystem, Intake intakeSubsystem, double indexerSpeed, double flywheelSpeed, double backspinnerSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootSubsystem);
    addRequirements(intakeSubsystem);
    this.indexerSpeed=indexerSpeed;
    this.flywheelSpeed=flywheelSpeed;
    this.backspinnerSpeed=backspinnerSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Shooter.setBackspinnerSpeed(indexerSpeed);
    Shooter.setAcceleratorSpeed(flywheelSpeed);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Shooter.getAcceleratorSpeed()*Constants.acceleratorRPMToPercent >= flywheelSpeed && Shooter.getBackspinnerSpeed()*Constants.backspinnerRPMToPercent >= backspinnerSpeed) {
      Shooter.setIndexerSpeed(0.9);
      Intake.setKowalksiMotor(0.9);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.setIndexerSpeed(0);
    Shooter.setAcceleratorSpeed(0);
    Shooter.setBackspinnerSpeed(0);
    Intake.setKowalksiMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
