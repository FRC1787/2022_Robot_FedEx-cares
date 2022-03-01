package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoCamera.WhiteBalance;
import frc.robot.Constants;

public class Vision extends SubsystemBase {


  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public static double distToTarget = 0;

  public Vision() {

  }

  private static final int STANDARD_IMG_WIDTH = 160;
  private static final int STANDARD_IMG_HEIGHT = 120;
//   public void configureCamera(UsbCamera camera, boolean targetingCamera) {
//     camera.setResolution(STANDARD_IMG_WIDTH, STANDARD_IMG_HEIGHT);
//     camera.setFPS(15);
//     if (targetingCamera) {
//         camera.setExposureManual(5);
//     } else {
//         camera.setExposureAuto();
//     }

//     camera.setBrightness(40);
//     camera.setWhiteBalanceManual(WhiteBalance.kFixedIndoor);
// }

  /**
   * Returns the x value of any target seen by the Limelight
   * 
   * @return x value of target
   */
  public static double getLimelightX() {
    return table.getEntry("tx").getDouble(0.0);
  }

  /**
   * Returns the current mode of the Limelight's LEDs
   * 
   * @return current mode of the Limelight's LEDs
   */
  public static int getCameraLightState() {
    return (int) table.getEntry("ledMode").getDouble(3);
  }

  public static double calculateAcceleratorRPM() {
    //find regression formula here
    return 3600;
  }

  public static double calculateBackspinnerRPM() {
    return 3600;
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