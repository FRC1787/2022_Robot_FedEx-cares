// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  public static int extendState = 0; //0 = retracted, 1 = intermediate, 2 = extended
  public static boolean limitSwitchPreviousTick;
  public static CANSparkMax arm = new CANSparkMax(7, MotorType.kBrushless);
  public static DigitalInput limitSwitch = new DigitalInput(0);
  public static DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);

  public Climb() {
    arm.setIdleMode(IdleMode.kBrake);
    piston.set(DoubleSolenoid.Value.kForward); //forward channel pushes up
    limitSwitchPreviousTick = limitSwitch.get();
  }

  //these functions are just for readability
  public static boolean isExtended() {
    return (extendState == 2);
  }
  public static boolean isRetracted() {
    return (extendState == 0);
  }

  //positive value = retracting, negative value = extending
  public static void setArm(double speed) {
    arm.set(speed);
    
  }

  @Override
  public void periodic() {
    //continually updates extendState using the reed switch and two magnets on each end of the arm
    if (!limitSwitch.get() && limitSwitchPreviousTick) { //if limit switch changes from not activated to activated
      if (arm.get() > 0) { //if arm is extending
        extendState = 2;
      }
      else if (arm.get() < 0) { //arm is retracted all the way
        extendState = 0;
      }
    }
    else if (limitSwitch.get() && !limitSwitchPreviousTick) {
      extendState = 1; //if limit switch deactivates, arm is intermediate mode
    }
    limitSwitchPreviousTick = limitSwitch.get();
    SmartDashboard.putBoolean("isExtended", isExtended());
    SmartDashboard.putBoolean("isRetracted", isRetracted());
    SmartDashboard.putBoolean("limitSwitchGet", limitSwitch.get());
    SmartDashboard.putNumber("armget", arm.get());
  }
}
