// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TestShoot extends CommandBase {
  /** Creates a new TestShoot. */
  PIDController flywheelPID = new PIDController(Constants.kpShooter, Constants.kiShooter, Constants.kdShooter);
  PIDController backspinnerPID = new PIDController(Constants.kpShooter, Constants.kiShooter, Constants.kdShooter);

  private double flywheelSetpoint;
  private double backspinnerSetpoint;
  
  public TestShoot(Shooter shooter, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(intake);
    flywheelPID.setTolerance(150); //TODO: adjust this for better 2nd shot
    backspinnerPID.setTolerance(150);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheelSetpoint = Shooter.flywheelCustom();
    backspinnerSetpoint = Shooter.backspinnerCustom();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.setFlywheelRPM(
      (flywheelPID.calculate(Shooter.getFlywheelSpeed(), flywheelSetpoint)
      + Constants.kfShooter*flywheelSetpoint) //basic feedforward
    );

    Shooter.setBackspinnerRPM(
      (backspinnerPID.calculate(Shooter.getBackspinnerSpeed(), backspinnerSetpoint)
      + Constants.kfShooter*backspinnerSetpoint) //basic feedforward
    );


    if (flywheelPID.atSetpoint() && backspinnerPID.atSetpoint()) {
      Shooter.setIndexerSpeed(0.3);
      Intake.setKowalskiMotor(0.6);
      Intake.setIntakeMotor(-0.6);
    }
    else {
      Shooter.setIndexerSpeed(0);
      Intake.stopAllMotors();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.stopAllMotors();
    Intake.stopAllMotors();
    Shooter.raiseShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}