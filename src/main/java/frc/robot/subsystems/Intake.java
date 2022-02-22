// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private static CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);
  private static CANSparkMax kowalski    = new CANSparkMax(Constants.kowalksiMotorID, MotorType.kBrushless);
  //REPLACE THESE VALUES
  private static DoubleSolenoid leftPiston  = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.leftIntakePistonFowardID, Constants.leftIntakePistonReverseID); 
  private static DoubleSolenoid rightPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.rightIntakePistonFowardID, Constants.rightIntakePistonReverseID);

  public Intake() {
    leftPiston.set(DoubleSolenoid.Value.kForward);
    rightPiston.set(DoubleSolenoid.Value.kForward); //depends on how the ids are set up

  }

  public static void setIntake(DoubleSolenoid.Value state) {
    leftPiston.set(state);
    rightPiston.set(state);
  }

  public static void setIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }

  public static void setKowalksiMotor(double speed) {
    kowalski.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
