package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class Arm extends SubsystemBase {

    SparkFlex m_armSpark = new SparkFlex(CANIds.kTestarmMotor, MotorType.kBrushless);
    SparkClosedLoopController m_controller = m_armSpark.getClosedLoopController();
    private double targetposition = 0;
    private double targetvelocity = 0;
    SlewRateLimiter velocityLimiter = new SlewRateLimiter(180.0);
    public static final double Floorarm = -105;
    public static final double Reefarm = -50;
    public static final double Spitarm = -10;
    public static final double Middlearm = 0;
    public static final double SoftLimitMax = 12;
    public static final double SoftLimitMin = -180.0;
    public static final double SoftSpeedLimit = 20;
    public static final double L2Arm = -83;
    public static final double L3Arm = -95;
    public static final double CoralDrop = -56;
    public static final double BallonCoral = -75;

    public Arm() {

        SparkFlexConfig config = new SparkFlexConfig();
    
        config.encoder.positionConversionFactor(2.88); //arm degrees = motor rotations (default units) x 360 deg/rotations x (1/5) x (1/5) x (1/5)
        config.encoder.velocityConversionFactor(0.048); // arm degrees/sec = motor rotations/minute (default units) x 360 deg/rotations x (1/60) minutes/sec x (1/5) x (1/5) x (1/5)
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimitEnabled(true);
        config.softLimit.forwardSoftLimit(SoftLimitMax);
        config.softLimit.reverseSoftLimit(SoftLimitMin);

        // Set PID gains
        config.closedLoop
            // Position control in slot 0
            .p(0.01, ClosedLoopSlot.kSlot0)
            .i(0, ClosedLoopSlot.kSlot0)
            .d(0, ClosedLoopSlot.kSlot0)
            .outputRange(-0.3, 0.3, ClosedLoopSlot.kSlot0)
            // Velocity control in slot 1
            .p(0.005, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .velocityFF(0.0018, ClosedLoopSlot.kSlot1)
            .outputRange(-0.3, 0.3, ClosedLoopSlot.kSlot1);
        
        m_armSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void Periodic(){
        SmartDashboard.putNumber("Relative Arm Position (deg)", m_armSpark.getEncoder().getPosition());
    }

    public Command ArmtopositionCommand(double Position){
        return this.runOnce(() -> this.ArmtoPosition(Position));
    }
public boolean ArmSafeTravel(){
//if (targetposition >= Spitarm&&targetposition <= Middlearm)
{
    return true;
}
//return false;
}
    private void ArmtoPosition(double Position){
        ResetVelocityControl(); // reset velocity controller when switching to position
        m_controller.setReference(Position, ControlType.kPosition);
        targetposition = Position;
    }

    public void Nudge(double Nudgevalue){
        ResetVelocityControl(); // reset velocity controller when switching to position
        targetposition += Nudgevalue;
        targetposition = MathUtil.clamp(targetposition,SoftLimitMin,SoftLimitMax);
        m_controller.setReference(targetposition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void Run(double speed){
        targetposition = m_armSpark.getEncoder().getPosition(); // reset position controller when switching to velocity
        targetvelocity = MathUtil.clamp(speed, -SoftSpeedLimit, SoftSpeedLimit);
        targetvelocity = velocityLimiter.calculate(targetvelocity);
        m_controller.setReference(targetvelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }

    private void ResetVelocityControl(){
        targetvelocity = 0; 
        velocityLimiter.reset(0.0);
    }
}
