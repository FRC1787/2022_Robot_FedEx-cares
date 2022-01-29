// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean botMostLikelyToGetPHD = true;


    public static final double gearboxRatio = 10.38;
    public static final double wheelDiameter = 6.0;
    public static final int pulsesPerRotation = 42;

    public static final double ks = 0.20491;
    public static final double kv = 2.6341;
    public static final double ka = 0.69297;

    public static final double kp = 2.5183;

    public static final double kTrackwidth = 0.66; //idk if this is right lole
    public static final DifferentialDriveKinematics kDriveKinematics = 
        new DifferentialDriveKinematics(kTrackwidth);
    
    public static final double kMaxSpeed = 1.5;
    public static final double kMaxAcceleration = 1.5;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double autoMaxVoltage = 10;

    // Measurements for limelight
    public static final double tapeHeight = 81;
    public static final double limelightHeight = 46;
    public static final double limelightAngle = 25;


    // button ids
    public static final int toggleLimelightButtonID = 12;
    public static final int lookToTargetButtonID = 5;
}
