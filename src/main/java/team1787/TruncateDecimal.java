// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1787;

import java.math.BigDecimal;
import java.math.RoundingMode;

/** Add your docs here. */
public class TruncateDecimal {
  public static BigDecimal truncateDecimal(double x,int numberofDecimals)
{
    if ( x > 0) {
        return new BigDecimal(String.valueOf(x)).setScale(numberofDecimals, RoundingMode.FLOOR);
    } else {
        return new BigDecimal(String.valueOf(x)).setScale(numberofDecimals, RoundingMode.CEILING);
    }
}
}
