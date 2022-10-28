// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1787.TrunkOrTreat;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class SpookyButton extends Button {
  public static DigitalInput ghostButton = new DigitalInput(8);

  public static boolean prevState = ghostButton.get();
  public SpookyButton() {
    prevState = ghostButton.get();
  }

  
  @Override
  public boolean get() {
    if (ghostButton.get() != prevState) {
      System.out.println("\n\n" + prevState + "\n\n");
      prevState = ghostButton.get();
      System.out.println("\n\ndifference\n\n");
      
      return true;
    }
    return false;
  }

}
