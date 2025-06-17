package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ZombieMarch extends Command {
    private final DriveSubsystem drive;
    private Timer timer = new Timer();
    boolean done;

    public ZombieMarch(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();        
        done = false;
    }

    @Override
    public void execute() {
        double time = timer.get();
        
        if (time < 41.586) stop();
        // forward
        else if (time < 42.636) marchright();
        else if (time < 43.564) marchleft();
        else if (time < 44.664) marchright();
        else if (time < 45.616) marchleft();
        // take it back
        else if (time < 46.692) backright();
        else if (time < 47.658) backleft();
        else if (time < 48.724) backright();
        else if (time < 49.739) backleft();
        // to the front
        else if (time < 50.743) marchright();
        else if (time < 51.766) marchleft();
        else if (time < 52.781) marchright();
        else if (time < 53.721) marchleft();
        // take it back
        else if (time < 54.810) backright();
        else if (time < 55.740) backleft();
        else if (time < 56.838) backright();
        else if (time < 57.770) backleft();
        
        else done=true;
    }


    private void stop() {
        drive.drive(0,0,0,false);
    }

    private void marchright() {
        drive.drive(0.05, 0.0, 0.05, false);
    }

    private void marchleft() {
        drive.drive(0.0, 0.0, -0.05, false);
    }

    private void backright() {
        drive.drive(-0.05, 0.0, -0.05, false);
    }

    private void backleft() {
        drive.drive(0.0, 0.0, 0.05, false);
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}

