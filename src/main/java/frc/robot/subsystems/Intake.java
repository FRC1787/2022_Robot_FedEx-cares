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
  private static DoubleSolenoid intakePiston  = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.leftIntakePistonFowardID, Constants.leftIntakePistonReverseID); 

  public Intake() {
    setRampRate(1);
    setIntake(DoubleSolenoid.Value.kReverse);

  }

  public static void setAmpLimit(int limit) {
    kowalski.setSmartCurrentLimit(limit);
    intakeMotor.setSmartCurrentLimit(limit);
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
    intakePiston.set(state);
  }


  /**
   * Toggles intake solenoid.
   * Equivalent to {@code solenoid.toggle()}
   */

  public static void toggleIntake() {
    intakePiston.toggle();
  }

  /**
   * Sets the speed of the intake motor
   * 
   * <p>
   * To set the speed of the motor, input a double between 0 and 1.0. For example,
   * {@code setIntakeMotor(0.5);}
   * </p>
   * @param speed - Speed to set for the motor. Value should be between 0 and 1.0
   */
  public static void setIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }

  public static void setKowalksiMotor(double speed) {
    kowalski.set(speed);
  }

  public static void setRampRate(double rate) {
    intakeMotor.setOpenLoopRampRate(rate);
    kowalski.setOpenLoopRampRate(rate);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
