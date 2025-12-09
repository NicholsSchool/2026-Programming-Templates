package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
    
    private IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private double targetPosition = 0.0;
    private double manualVoltage = 0.0;
    private boolean atTargetPosition = false;
    private IndexerState state;
    
    // PID Controller used for shifting by a set amount.
    private final ProfiledPIDController PIDController =
    new ProfiledPIDController(Constants.IndexerConstants.kP, Constants.IndexerConstants.kI, Constants.IndexerConstants.kD,
    new TrapezoidProfile.Constraints(Constants.IndexerConstants.kMaxVel, Constants.IndexerConstants.kMaxAccel));
    
    public Indexer(IndexerIO io) {
        
        this.io = io;
        setTargetPos(inputs.indexerPositionIN);
        
    }
    
    private enum IndexerState {
        MANUAL,
        POSITION
    }
    
    public enum ShiftDirection {
        OUT(1),
        IN(-1);
        
        public final int multiplier;
        
        private ShiftDirection(int multiplier) {
            this.multiplier = multiplier;
        }
    }
    
    @Override
    public void periodic(){
        io.updateInputs(inputs);
        
        Logger.processInputs("Elevator", inputs);

        double voltage = 0.0;
        
        switch(state){
            case POSITION:
            voltage = PIDController.calculate(inputs.indexerPositionIN);
            break;
            case MANUAL:
            voltage = manualVoltage;
            setTargetPos(inputs.indexerPositionIN);
        }
        
        if (!atTargetPosition) {
            atTargetPosition = PIDController.atGoal();
            if (atTargetPosition) System.out.println("Indexer Shift Completed.");
        }
        
        io.setMotorVoltage(voltage);
    }
    
    public void setVoltage(double voltage) {
        this.manualVoltage = voltage;
    }
    
    public void setTargetPos(double targetPosition) {
        this.targetPosition = targetPosition;
        PIDController.setGoal((targetPosition));
        PIDController.reset(inputs.indexerPositionIN);
        atTargetPosition = false;
    }
    
    public Command runGoToPosCommand(double targetPosition){
        state = IndexerState.POSITION;
        System.out.println("Indexer TargetPosition is now:" + targetPosition );
        return new InstantCommand(() -> setTargetPos(targetPosition), this);
    }
    
    public void startGoToPos(double targetPosition){
        state = IndexerState.POSITION;
        System.out.println("Starting Shift to:" + targetPosition );
        setTargetPos(targetPosition);
    }
    
    public Command shiftCommand(ShiftDirection dir) {
        double shiftAmountIN = IndexerConstants.shiftLengthIN * dir.multiplier;
        return new FunctionalCommand(
        () -> startGoToPos(this.targetPosition + shiftAmountIN),
        () -> state = IndexerState.POSITION,
        interrupted -> setTargetPos(inputs.indexerPositionIN),
        () -> this.hasReachedTarget(),
        this);
    }
    
    @AutoLogOutput
    public boolean hasReachedTarget() {
        return PIDController.atGoal();
    }
}