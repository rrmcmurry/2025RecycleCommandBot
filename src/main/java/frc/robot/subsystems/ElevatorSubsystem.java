package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

public class ElevatorSubsystem extends SubsystemBase{

    private final SparkMax m_ElevatorLeftSpark; 
    private final SparkMax m_ElevatorRightSpark;
    private RelativeEncoder encoder;
    private double currentspeed;
    private double currentposition;
    private double previousposition;
    public double elevatorspeedlimiter;
   

    public static final class ElevatorConstants {
        // SPARK MAX CAN IDs
        public static final int kElevatorLeftCanId = 9;
        public static final int kElevatorRightCanId = 10;

        // Speed
        public static final double kElevatorSpeed = 1.0;

        public static final double kLowestLevel = 0.0;
        public static final double kHighestLevel = 100.0;

        public static final SparkMaxConfig leadConfig = new SparkMaxConfig();
        public static final SparkMaxConfig followConfig = new SparkMaxConfig();

        static {              
            leadConfig.smartCurrentLimit(50);
            leadConfig.idleMode(IdleMode.kBrake);  
            leadConfig.openLoopRampRate(2.0);   
            leadConfig.closedLoopRampRate(0.0);   
            leadConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            leadConfig.closedLoop.pid(0.4, 0, 0.2);
            leadConfig.closedLoop.outputRange(-1,1);
            followConfig.apply(leadConfig);
            followConfig.inverted(true);
        }
    } 

    public ElevatorSubsystem(){

        // Left Elevator Motor 
        m_ElevatorLeftSpark = new SparkMax(ElevatorConstants.kElevatorLeftCanId, MotorType.kBrushless);
        m_ElevatorLeftSpark.configure(ElevatorConstants.leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Right Elevator Motor  
        ElevatorConstants.followConfig.follow(m_ElevatorLeftSpark, true);       
        m_ElevatorRightSpark = new SparkMax(ElevatorConstants.kElevatorRightCanId, MotorType.kBrushless);   
        m_ElevatorRightSpark.configure(ElevatorConstants.followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        // Elevator Encoder
        encoder = m_ElevatorLeftSpark.getEncoder();
    }

    public void init() {
        // Configure right Elevator Motor to follow left just in case this was missed at startup        
        ElevatorConstants.followConfig.follow(m_ElevatorLeftSpark, true);
        m_ElevatorRightSpark.configure(ElevatorConstants.followConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }


    private double scaledSpeedToTop() {        
        return ElevatorConstants.kElevatorSpeed * Math.min(150,(ElevatorConstants.kHighestLevel - currentposition))/150; 
    }
    
    private double scaledSpeedToBottom() {        
        return -ElevatorConstants.kElevatorSpeed * Math.min(100, currentposition)/100; 
    }

    public void robotPeriodic() {
        currentposition = encoder.getPosition();        
        // Speed limiter used to limit swerve drive speed based on elevator height to prevent tipping with a higher center of gravity
        elevatorspeedlimiter = (ElevatorConstants.kHighestLevel + 70 - currentposition) / (ElevatorConstants.kHighestLevel + 70); 
    }

    public void raise() {
        currentposition = encoder.getPosition();
        if (currentposition < ElevatorConstants.kHighestLevel) {
            currentspeed = scaledSpeedToTop();
            m_ElevatorLeftSpark.set(currentspeed);}
        else {
            m_ElevatorLeftSpark.stopMotor();}
    }

    public void lower() {    
        currentposition = encoder.getPosition();    
        if (currentposition > ElevatorConstants.kLowestLevel) {
            currentspeed = scaledSpeedToBottom();
            m_ElevatorLeftSpark.set(currentspeed);}
        else {
            m_ElevatorLeftSpark.stopMotor();}
    }

    public void stop() {
        m_ElevatorLeftSpark.stopMotor(); 
    }

    public void testPeriodic() {
        currentposition = encoder.getPosition();
        if (previousposition != currentposition)
            System.out.println("Elevator position: " + currentposition);
        previousposition = currentposition;
    }

    public Command raiseCommand() {
        return Commands.startEnd(this::raise, this::stop, this);
    }

    public Command lowerCommand() {
        return Commands.startEnd(this::lower, this::stop, this);
    }
}
