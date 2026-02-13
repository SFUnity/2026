package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ShooterVisualizer {
  private final LoggedMechanism2d hoodMechanism;
  private final LoggedMechanismRoot2d hoodRoot;
  private final LoggedMechanism2d turretMechanism;
  private final LoggedMechanismRoot2d turretRoot;
  private final LoggedMechanismLigament2d turret;
  private final LoggedMechanismLigament2d hood;
  private LoggedMechanismLigament2d shooterVisualizer;
  private double xCoord = 0; // TODO update
  private double yCoord = 0;
  private double zCoord = 0;
  private final String key;

  private final double turretDiameter = Units.inchesToMeters(8);

  public ShooterVisualizer(String key, Color color) {
    this.key = key;

    hoodMechanism = new LoggedMechanism2d(0, 0, new Color8Bit(Color.kBlack));
    hoodRoot = hoodMechanism.getRoot("Shooter Root", xCoord, yCoord);
    hood = new LoggedMechanismLigament2d("Hood", 0, 0);

    turretMechanism = new LoggedMechanism2d(0, 0, new Color8Bit(Color.kBlack));
    turretRoot = turretMechanism.getRoot("Turret Root", xCoord, yCoord);
    turret = new LoggedMechanismLigament2d("Turret", turretDiameter, 0, 8,  new Color8Bit(color));

    turretRoot.append(turret);
    hoodRoot.append(hood);
  }

  public void update(double turretAngle, double hoodAngle) {
    turret.setAngle(turretAngle);
    hood.setAngle(hoodAngle);
    Logger.recordOutput("Subsystems/Shooter/Hood/Mechanism2D/" + key, hoodMechanism);

    Pose3d shooterPose =
        new Pose3d(
            xCoord + Math.cos(turretAngle) * Math.cos(hoodAngle),
            yCoord + Math.sin(turretAngle) * Math.cos(hoodAngle),
            zCoord + Math.sin(hoodAngle),
            new Rotation3d());
    Logger.recordOutput("Subsystems/Shooter/Mechanism3D/" + key, shooterPose);
  }
}
