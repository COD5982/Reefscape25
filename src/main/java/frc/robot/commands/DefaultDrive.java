package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainBase;

public class DefaultDrive extends Command {
    private final DriveTrainBase m_drive;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;
    private final DoubleSupplier m_rotSpeed;
    private final BooleanSupplier m_turbo;

    public DefaultDrive(DriveTrainBase subsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed, BooleanSupplier turbo) {
        m_drive = subsystem;
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        m_rotSpeed = rotSpeed;
        m_turbo = turbo;
        addRequirements(m_drive);
     }

    @Override
    public void execute() {
        double xspeed=m_xSpeed.getAsDouble();
      
        double yspeed=m_ySpeed.getAsDouble();
       
        double rotspeed=m_rotSpeed.getAsDouble();
      
        boolean turbo =m_turbo.getAsBoolean();
        if (!turbo){
            xspeed*=0.3;
            yspeed*=0.3;
            rotspeed*=0.3;
        }
        m_drive.drive(xspeed, yspeed, rotspeed, false);
    }
}
