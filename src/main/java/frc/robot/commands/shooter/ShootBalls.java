// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ShootBalls extends CommandBase {
  /** Creates a new ShootBalls. */
  PIDController flywheelPID = new PIDController(Constants.kpShooter, Constants.kiShooter, Constants.kdShooter);
  PIDController backspinnerPID = new PIDController(Constants.kpShooter, Constants.kiShooter, Constants.kdShooter);

  private double flywheelSetpoint;
  private double backspinnerSetpoint;

  public ShootBalls(Shooter shootSubsystem, Vision cameraSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootSubsystem);
    addRequirements(cameraSubsystem);
    flywheelPID.setTolerance(100);
    backspinnerPID.setTolerance(100);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheelSetpoint = Vision.calculateFlywheelRPM(); //function will change depending on threshold
    backspinnerSetpoint = Vision.calculateBackspinnerRPM();
    //TODO: have vision function to return shooter position
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
      Intake.setKowalksiMotor(0.4);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.setFlywheelSpeed(0);
    Shooter.setIndexerSpeed(0);
    Shooter.setBackspinnerSpeed(0);
    Intake.setKowalksiMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}