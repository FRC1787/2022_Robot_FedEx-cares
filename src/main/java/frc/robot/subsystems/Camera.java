package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Constants;

public class Camera extends SubsystemBase {

  public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  double h1 = 35.5;
  double h2 = 57;
  double a1 = 33;

  public Camera() {

  }

  public static double getLimelightX() {
    return table.getEntry("tx").getDouble(0.0);
  }

  public static int getCameraLightState() {
    return (int) table.getEntry("ledMode").getDouble(3);
  }

  @Override
  public void periodic() {

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double d = ((Constants.tapeHeight - Constants.limelightHeight)/(Math.tan(Math.toRadians(y + Constants.limelightAngle))));

    SmartDashboard.putNumber("Limelight Distance", d);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

  }
}