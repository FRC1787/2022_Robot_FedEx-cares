// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class OneEighty extends CommandBase {
  /** Creates a new OneEighty. */

  //TODO: make this use gyro this is horrible and the worst

  PIDController controller = new PIDController(0.05, 0, 0);
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ksAuto, Constants.kvAuto, Constants.kaAuto);
  public OneEighty(Drivetrain drivetrain, Vision vision) {
    controller.setTolerance(0.5);
    addRequirements(drivetrain);
    addRequirements(vision);

  }
  boolean finished = false;
  double tx = 0.0;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Drivetrain.resetGyro();
    tx = Drivetrain.getHeading() * -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  // a .70 to .80 timeout is ideal
  @Override
  public void execute() {   
    Drivetrain.tankDrive(
       3,
      -3
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
    return finished;
  }
}
