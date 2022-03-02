// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private static CANSparkMax arm = new CANSparkMax(Constants.armMotorID, MotorType.kBrushless);
  private static DigitalInput bottomLimitSwitch = new DigitalInput(0);
  private static DigitalInput topLimitSwitch    = new DigitalInput(1);
  private static DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.armPistonFowardID, Constants.armPistonReverseID);

  public Climb() {
    arm.setIdleMode(IdleMode.kBrake);
    piston.set(DoubleSolenoid.Value.kReverse); //reverse channel pushes up
    setAmpLimit(50);
  }

  public static void setAmpLimit(int limit) {
    arm.setSmartCurrentLimit(limit);
  }

  /**
   * Tests if arm is extended
   * 
   * @return true if the arm is extended and false otherwise
   */
  public static boolean isExtended() {
    return !topLimitSwitch.get();
  }

  /**
   * Tests if arm is retracted
   * 
   * @return true if the arm is retracted and false otherwise
   */
  public static boolean isRetracted() {
    return !bottomLimitSwitch.get();
  }

  /**
   * Sets the speed of the climb arm
   * 
   * <p>
   * To extend the arm, input a negative value, for example:
   * {@code setArm(-0.5);}
   * </p>
   * <p>
   * To retract the arm, input a positive value, for example:
   * {@code setArm(0.5);}
   * </p>
   * 
   * @param speed - speed of the arm input as a voltage
   * 
   */
  public static void setArm(double speed) {
    arm.set(speed);
    
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
  public static void setPiston(DoubleSolenoid.Value value) {
    piston.set(value);
  }

  @Override
  public void periodic() {
    //continually updates extendState using the reed switch and two magnets on each end of the arm
    SmartDashboard.putBoolean("isExtended", isExtended());
    SmartDashboard.putBoolean("isRetracted", isRetracted());
    SmartDashboard.putBoolean("bottomLimitSwitchGet", bottomLimitSwitch.get());
    SmartDashboard.putBoolean("topLimitSwitchGet", topLimitSwitch.get());
    SmartDashboard.putNumber("armget", arm.get());
  }
}
