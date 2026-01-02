package frc.robot.subsystems.shooter;


import org.littletonrobotics.junction.AutoLog;


public interface ShooterIO {


  @AutoLog
  public static class ShooterIOInputs {
    public double velocityRPMs = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }


  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Set the motor voltage */
  public void setVoltage(double volts);

  public void stop();
}
