package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ZombieMarch extends Command {
    private final DriveSubsystem drive;
    private Timer timer = new Timer();
    private float startAngle;
    private int phase = 0;

    public ZombieMarch(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
        startAngle = 0;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        startAngle = drive.getHeading(); // Assuming this returns yaw in degrees
        phase = 0;
    }

    @Override
    public void execute() {
        double time = timer.get();

        switch (phase) {
            case 0:
                // Forward while turning left
                drive.drive(0.2, 0.0, 0.1, true);
                if (time > 1.5) phase++;
                break;
            case 1:
                // Rotate back to original heading
                double currentHeading = drive.getHeading();
                double error = startAngle - currentHeading;
                double kP = 0.01;
                drive.drive(0.0, 0.0, error * kP, true);
                if (Math.abs(error) < 2) phase++;
                break;
            case 2:
                // Do it again
                timer.reset();
                phase++;
                break;
            case 3:
                drive.drive(0.2, 0.0, 0.1, true);
                if (timer.get() > 1.5) phase++;
                break;
            case 4:
                // Rotate back again
                double heading = drive.getHeading();
                double err = startAngle - heading;
                drive.drive(0.0, 0.0, err * 0.01, true);
                if (Math.abs(err) < 2) phase++;
                break;
            default:
                drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return phase > 4;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
