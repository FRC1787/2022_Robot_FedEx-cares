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
  private static int extendState = 0; //0 = retracted, 1 = intermediate, 2 = extended
  private static CANSparkMax arm = new CANSparkMax(Constants.armMotorID, MotorType.kBrushless);
  private static DigitalInput bottomLimitSwitch = new DigitalInput(1);
  private static DigitalInput topLimitSwitch    = new DigitalInput(0);
  private static DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.armPistonFowardID, Constants.armPistonReverseID);

  public Climb() {
    arm.setIdleMode(IdleMode.kBrake);
    piston.set(DoubleSolenoid.Value.kReverse); //reverse channel pushes up
  }

  //these functions are just for readability
  public static boolean isExtended() {
    return !topLimitSwitch.get();
  }
  public static boolean isRetracted() {
    return !bottomLimitSwitch.get();
  }
  //positive value = retracting, negative value = extending
  public static void setArm(double speed) {
    arm.set(speed);
    
  }

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
    SmartDashboard.putNumber("extendState", extendState);
    SmartDashboard.putNumber("armget", arm.get());
  }
}
