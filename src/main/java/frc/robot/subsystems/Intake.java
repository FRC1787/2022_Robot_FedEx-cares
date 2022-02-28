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

  public Intake() {
    leftPiston.set(DoubleSolenoid.Value.kReverse);
    //depends on how the ids are set up

  }
  
  /**
  * Sets the state of the pneumatic cylinder via the connected solenoid
  * 
  * <p>
  * To extend the pneumatic cylinder, use {@code kForward} for the {@code value} parameter, for example: 
  * {@code setPiston(Value.kForward);}
  * </p>
  * <p>
  * To retract the pneumatic cylinder, use {@code kReverse} for the {@code value} parameter, for example:
  * {@code setPiston(Value.kReverse);}
  * 
  * @param value - {@code Value.kForward} or {@code Value.kReverse}
  */
  public static void setIntake(DoubleSolenoid.Value state) {
    leftPiston.set(state);
  }

  /** 
   * Sets the speed of the intake motor
  */
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
