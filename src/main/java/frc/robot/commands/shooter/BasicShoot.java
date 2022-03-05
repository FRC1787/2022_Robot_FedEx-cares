// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class BasicShoot extends CommandBase {


  double indexerSpeed;
  double flywheelRPM;
  double backspinnerRPM;
  Timer timer = new Timer();

  /**
   * TEMPORARY TEST COMMAND
   * @param indexerSpeed - motor percentage; positive is correct direction
   * @param flywheelRPM - RPM, negative is correct direction
   * @param backspinnerRPM - RPM, positive is correct direction
   */
  public BasicShoot(Shooter shootSubsystem, Intake intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootSubsystem);
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    flywheelRPM = Vision.calculateFlywheelRPM();
    backspinnerRPM = Vision.calculateBackspinnerRPM();
    Shooter.setFlywheelRPM(flywheelRPM);
    Shooter.setBackspinnerRPM(backspinnerRPM);
    timer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() > 1.0) {
      Intake.setKowalskiMotor(0.4);
      Shooter.setIndexerSpeed(0.2);
    }
    else {
      Intake.setKowalskiMotor(0);
      Shooter.setIndexerSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.setIndexerSpeed(0);
    Shooter.setFlywheelSpeed(0);
    Shooter.setBackspinnerSpeed(0);
    Intake.setKowalskiMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
