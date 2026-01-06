// package frc.robot.subsystems.arm;

// import org.littletonrobotics.junction.AutoLogOutput;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.DigitalInput;
// import frc.robot.Constants;
// import frc.robot.Constants.CAN;
// public class ArmIOReal implements ArmIO {

//     //depending on how many you have I guess?
//     private TalonFX lShoulder;
//     private TalonFX rShoulder;
//     private final CANcoder armEncoder;



//     public ArmIOReal(){
//         lShoulder = new TalonFX(CAN.LEFT_ARM_SHOULDER, "Arm");
//         rShoulder = new TalonFX(CAN.RIGHT_ARM_SHOULDER, "Arm");
//         armEncoder = new CANcoder(CAN.ARM_ENCODER, "Arm");
        

//         var talonConfig = new TalonFXConfiguration();
//         talonConfig.CurrentLimits.StatorCurrentLimit = Constants.ArmConstants.ArmCurrentLimit;
//         talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
//         talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//         lShoulder.getConfigurator().apply(talonConfig);
//         rShoulder.setControl(new Follower(CAN.LEFT_ARM_SHOULDER, false));
//     }

//     private double getAngle(){
//         return armEncoder.getPosition().getValueAsDouble();
//     }


    
//     @Override
//     public void updateInputs (ArmIOInputs inputs){
//         inputs.currentAmps = new double[] {lShoulder.getStatorCurrent().getValueAsDouble(), lShoulder.getStatorCurrent().getValueAsDouble()};
//         inputs.velocityRadPerSec = new double[]{lShoulder.getVelocity().getValueAsDouble(), rShoulder.getVelocity().getValueAsDouble()};
//         inputs.appliedVoltage = new double[]{lShoulder.getMotorVoltage().getValueAsDouble(), rShoulder.getMotorVoltage().getValueAsDouble()};

//     }

//     @Override
//     public void setVoltage (double voltage){
//         lshoulder.setVoltage(voltage);
//     }

//     @AutoLogOutput
//     public double getArmAngle (){
//         return armEncoder.getPosition().getValueAsDouble;
//     }


// }
