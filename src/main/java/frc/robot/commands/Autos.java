// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrainBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command DriveAndDump(DriveTrainBase driveSubsystem, Arm armSubsystem, Intake intakeSubsystem) {
    return Commands.sequence(
      new TimedDrive(driveSubsystem, 2, -0.2, 0, 0, true),
      armSubsystem.ArmtopositionCommand(Arm.Reefarm),
      new RunCommand(() -> intakeSubsystem.Run(1.0), intakeSubsystem)
    );
  }

  public static Command ScoreReef(Arm armSub) {
    return armSub.ArmtopositionCommand(Arm.Reefarm);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
