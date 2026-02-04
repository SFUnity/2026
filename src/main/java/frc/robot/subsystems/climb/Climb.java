package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {

  private ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  public Climb(ClimbIO io) {
    this.io = io;
    Logger.recordOutput("ClimbPosition", downMeters);
  }

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Climb", inputs);
    GeneralUtil.logSubsystem(this, "Climb");
  }

  public Command climbUp() {
    return run(() -> io.setPosition(upMeters))
        .alongWith(runOnce(() -> Logger.recordOutput("ClimbPosition", upMeters)));
  }

  public Command climbDown() {
    return run(() -> io.setPosition(downMeters))
        .alongWith(runOnce(() -> Logger.recordOutput("ClimbPosition", downMeters)));
  }
}
