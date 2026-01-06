package frc.robot.subsystems.arm;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double angleRads = 0.0;
        public double angleDegs = 0.0;
        public double[] currentAmps = {0.0, 0.0};
        public double[] velocityRadPerSec = {0.0, 0.0};
        public double[] appliedVoltage = {0.0, 0.0};
    }

    public default void updateInputs(ArmIOInputs inputs) {}
    public default void setVoltage(double voltage) {}
    
}

