// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;
import java.util.concurrent.TimeUnit;
import edu.wpi.first.wpilibj.DoubleSolenoid;


public class ClimbRoutine extends CommandBase {
  /** Creates a new ClimbRoutine. */

  private int step;
  private int bar;

  public ClimbRoutine(Climb climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    step = 0;
    bar = 0;
    //mount limit switches so it is activated when arm is fully extended/retracted
    //assumes arm is already on bar when command is started (make sure that bottom limit switch is not active)
    //retract arm until bottom limit switch is met
    //see https://docs.revrobotics.com/sparkmax/feature-description/data-port#:~:text=SPARK%20MAX%20has%20two%20limit,output%20into%20the%20neutral%20state.
    //and https://robotpy.readthedocs.io/projects/rev/en/stable/rev/CANSparkMax.html#rev.CANSparkMax.setSmartCurrentLimit
    //retract pneumatic
    //extend arm until top limit switch met
    //extend pneumatic
    //retract arm until bottom limit switch met
    //repeat steps for 15-point climb
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (bar < 2) {
      switch (step) {

        case (0):
          Climb.setArm(-0.2); //retract arm until limit switch activates
          if (Climb.isRetracted()) {
            Climb.setArm(0); //stop arm and move to next step
            step++;
            break;
          }

        case (1):
          Climb.piston.set(DoubleSolenoid.Value.kReverse);
          step++;
          break;

        case (2):
          Climb.setArm(0.5); //extend arm until limit switch activates
          if (Climb.isExtended()) {
            Climb.setArm(0);
            Climb.piston.set(DoubleSolenoid.Value.kForward);
            step++;
          }
          break;
        
        case (3):
          Climb.setArm(-0.2);
          if (Climb.isRetracted()) {
            Climb.setArm(0);
            step++;
            break;
          }
          

        case (4):
          step = 0;
          bar++;
          break;

      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Climb.setArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return bar >= 2;
  }
}
