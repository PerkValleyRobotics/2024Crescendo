// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSubsystem;

public class ToggleAmpCmd extends Command {

  private AmpSubsystem ampy;
  private double angle;
  /** Creates a new ToggleAmpCmd. */
  public ToggleAmpCmd(AmpSubsystem ampy, DoubleSupplier angle) {

    this.ampy = ampy;
    this.angle = angle.getAsDouble();

    addRequirements(ampy);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // ampy.setServoReference(angle);
    ampy.toggleServo();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (-3 < ampy.getAngle() && ampy.getAngle() < 3) ampy.setServoReference(0.666);
    // else ampy.setServoReference(0);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
