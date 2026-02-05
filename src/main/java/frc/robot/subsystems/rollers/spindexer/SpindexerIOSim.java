package frc.robot.subsystems.rollers.spindexer;

public class SpindexerIOSim implements SpindexerIO {

  private double appliedVolts = 0;

  public SpindexerIOSim() {}

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void run(double voltage) {
    appliedVolts = voltage;
  }
}
