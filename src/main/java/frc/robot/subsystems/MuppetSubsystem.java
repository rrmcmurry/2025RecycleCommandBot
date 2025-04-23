package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MuppetSubsystem extends SubsystemBase{
    
    Servo lefthandservo;
    Servo righthandservo;

    public MuppetSubsystem() {
        lefthandservo = new Servo(0);
        righthandservo = new Servo(1);
    }

    public void drivestick(double forward) {
        double servovalue = (forward + 1.0)/2.0;  //Convert values from -1 thru +1 to 0 thru +1
        lefthandservo.set(servovalue);
    }

    public void elevatorstick(double value) {
        righthandservo.set(value);
    }
}
