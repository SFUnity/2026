package frc.robot.subsystems.shooter.flywheels;

import frc.robot.util.LoggedTunableNumber;

public class FlywheelsConstants {
  public static final int leaderID = 0;
  public static final int followID = 0;

  public static final LoggedTunableNumber readyRPMSetpoint =
      new LoggedTunableNumber("Flywheels/readyVolts", 100);

  public static final LoggedTunableNumber flywheelTolerance =
      new LoggedTunableNumber("Flywheels/tolerance", 5); // TODO tune
}
