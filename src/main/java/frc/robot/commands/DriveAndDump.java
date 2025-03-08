package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainBase;

public class DriveAndDump extends Command {

    DriveTrainBase m_drive;
    double timeOut, x, y, rot;
    Boolean field;
    Timer timer;

    public DriveAndDump(DriveTrainBase subsystem, double duration, double xSpeed, double ySpeed, double rotSpeed, Boolean fieldRel) {
        m_drive = subsystem;
        timeOut = duration;
        x = xSpeed;
        y = ySpeed;
        rot = rotSpeed;
        field = fieldRel;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        timer.start();
    }

    @Override
    public void execute() {
        m_drive.drive(x, y, rot, field);
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= timeOut;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, true);
    }
}