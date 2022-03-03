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


  double indexerSpeed;
  double flywheelRPM, backspinnerRPM;

  /**
   * TEMPORARY TEST COMMAND
   * @param indexerSpeed - motor percentage; positive is correct direction
   * @param flywheelRPM - RPM, negative is correct direction
   * @param backspinnerRPM - RPM, positive is correct direction
   */
  public BasicShoot(Shooter shootSubsystem, Intake intakeSubsystem, double indexerSpeed, double flywheelRPM, double backspinnerRPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootSubsystem);
    addRequirements(intakeSubsystem);
    this.indexerSpeed=indexerSpeed;
    this.flywheelRPM=flywheelRPM;
    this.backspinnerRPM=backspinnerRPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Shooter.setFlywheelRPM(flywheelRPM);
    Shooter.setBackspinnerRPM(backspinnerRPM);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Shooter.getFlywheelSpeed() >= flywheelRPM && Shooter.getBackspinnerSpeed() >= backspinnerRPM) {
      Shooter.setIndexerSpeed(indexerSpeed);
      Intake.setKowalksiMotor(indexerSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.setIndexerSpeed(0);
    Shooter.setFlywheelSpeed(0);
    Shooter.setBackspinnerSpeed(0);
    Intake.setKowalksiMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
