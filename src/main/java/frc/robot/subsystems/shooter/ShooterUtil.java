package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseManager;

public class ShooterUtil {

  private final LoggedTunableNumber phaseDelay =
      new LoggedTunableNumber("Shooter/PhaseDelay", 0.03);
  private final PoseManager poseManager;

  private final InterpolatingDoubleTreeMap launchHoodAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  public ShooterUtil(PoseManager poseManager) {
    this.poseManager = poseManager;
  }

  public record LaunchingParameters(
      boolean isValid,
      double turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  public LaunchingParameters getLaunchingParameters(Pose3d targetPose) {
    Pose2d robotPose = poseManager.getPose();
    Twist2d robotVelocity = poseManager.getRobotVelocity();
    robotPose =
        robotPose.exp(
            new Twist2d(
                robotVelocity.dx * phaseDelay.get(), robotVelocity.dy * phaseDelay.get(), 0));
    LaunchingParameters params = new LaunchingParameters(false, 0, 0, 0, 0, 0);
    return params;
  }
}
