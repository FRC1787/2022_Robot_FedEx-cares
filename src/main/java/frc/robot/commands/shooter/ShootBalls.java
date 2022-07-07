// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootBalls extends CommandBase {
  /** Creates a new ShootBalls. */
  PIDController flywheelPID = new PIDController(Constants.kpShooter, Constants.kiShooter, Constants.kdShooter);
  PIDController backspinnerPID = new PIDController(Constants.kpShooter, Constants.kiShooter, Constants.kdShooter);

  private double flywheelSetpoint;
  private double backspinnerSetpoint;

  public ShootBalls(Shooter shootSubsystem, Intake intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootSubsystem);
    addRequirements(intakeSubsystem);
    flywheelPID.setTolerance(100); //TODO: adjust this for better 2nd shot
    backspinnerPID.setTolerance(100);
    flywheelPID.setIntegratorRange(0, 175);
    backspinnerPID.setIntegratorRange(0, 175);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flywheelSetpoint = Vision.calculateFlywheelRPM();
    backspinnerSetpoint = Vision.calculateBackspinnerRPM();

    flywheelPID.setSetpoint(flywheelSetpoint);
    backspinnerPID.setSetpoint(backspinnerSetpoint);

    Shooter.setFlywheelRPM(
      MathUtil.clamp(flywheelPID.calculate(Shooter.getFlywheelSpeed()), -100, 100)
      + Constants.kfShooter*flywheelSetpoint
    );
    

    Shooter.setBackspinnerRPM(
      MathUtil.clamp(backspinnerPID.calculate(Shooter.getBackspinnerSpeed()), -100, 100)
      + Constants.kfShooter*backspinnerSetpoint
    );


    if (flywheelPID.atSetpoint() && backspinnerPID.atSetpoint()) {
      Shooter.setIndexerSpeed(0.4); //make this pid
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
    //return (Shooter.isRaised && Vision.getLimelightA() == 0);
    return false;
  }
}
