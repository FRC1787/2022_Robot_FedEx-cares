package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Constants;

public class Camera extends SubsystemBase {


  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public static double distToTarget;

  public Camera() {

  }

  public static double getLimelightX() {
    return table.getEntry("tx").getDouble(0.0);
  }

  public static int getCameraLightState() {
    return (int) table.getEntry("ledMode").getDouble(3);
  }

  public static double calculateAcceleratorSpeed() {
    //find regression formula here
    return 0;
  }

  public static double calculateBackspinnerSpeed() {
    return 0;
  }

  @Override
  public void periodic() {

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    distToTarget = ((Constants.tapeHeight - Constants.limelightHeight)/(Math.tan(Math.toRadians(y + Constants.limelightAngle))));

    SmartDashboard.putNumber("Limelight Distance", distToTarget);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

  }


}