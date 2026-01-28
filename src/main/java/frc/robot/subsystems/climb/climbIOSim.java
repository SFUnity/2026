package frc.robot.subsystems.climb;

public class climbIOSim implements climbIO {

  private double appliedVolts = 0;

  public climbIOSim() {}

  @Override
  public void updateInputs(climbIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void runVolts(double voltage) {
    appliedVolts = voltage;
  }
}
