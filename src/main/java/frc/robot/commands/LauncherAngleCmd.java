// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

public class LauncherAngleCmd extends Command {
  /** Creates a new LauncherAngleCmd. */
  LauncherSubsystem launcher;
  DoubleSupplier setPos;


  public LauncherAngleCmd(LauncherSubsystem launcher, DoubleSupplier setPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcher = launcher;
    this.setPos = setPos;
    addRequirements(launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(setPos.getAsDouble() <= 8.55 && setPos.getAsDouble() >= -2)launcher.setAngle(setPos.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return launcher.getAngle() >= setPos.getAsDouble()-.05 && launcher.getAngle() <= setPos.getAsDouble()+.1;
  }
}
