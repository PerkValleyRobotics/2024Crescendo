// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

public class LauncherManualAngleCmd extends Command {
  /** Creates a new LauncherAngleCmd. */
  LauncherSubsystem launcher;
  double setPos;


  public LauncherManualAngleCmd(LauncherSubsystem launcher, double setPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcher = launcher;
    this.setPos = setPos;
    addRequirements(launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcher.setAngle(launcher.getAngle() + setPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
