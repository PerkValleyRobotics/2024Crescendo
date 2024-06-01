// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class RunLauncherCmd extends Command {
  private double speed;
  private final LauncherSubsystem shooter;
  private boolean PID;
  /** Creates a new RunIntakeCmd. */
  public RunLauncherCmd(LauncherSubsystem shooter, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.speed = speed.getAsDouble();
    this.PID = false;

    addRequirements(shooter);
  }
  public RunLauncherCmd(LauncherSubsystem shooter, DoubleSupplier speed, boolean PID) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.speed = speed.getAsDouble();
    this.PID = PID;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Start the motor running at the specified Speed
    if (PID) shooter.setReference(speed);
    else shooter.setSpeed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Stop the motor
    shooter.setLeft(0);
    shooter.setRight(0);
    shooter.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
