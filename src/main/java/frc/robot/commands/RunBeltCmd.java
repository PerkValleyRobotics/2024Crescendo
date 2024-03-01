// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;

public class RunBeltCmd extends Command {
  /** Creates a new RunIntakeCmd. */
  ConveyorSubsystem conveyor;
  double speed;
  double minVel;
  double time;

  private Timer timer = new Timer();

  public RunBeltCmd(ConveyorSubsystem conveyor, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.conveyor = conveyor;
    this.speed = speed;
    this.time = 0;
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    conveyor.runBelt(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.runBelt(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (time != 0 && timer.hasElapsed(time)) return true;
    return false;
  }
}
