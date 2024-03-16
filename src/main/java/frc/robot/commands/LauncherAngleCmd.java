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

  private boolean isAuto;
  private boolean spinUp;

  public LauncherAngleCmd(LauncherSubsystem launcher, DoubleSupplier setPos, boolean isAuto) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcher = launcher;
    this.setPos = setPos;
    this.isAuto = isAuto;
    this.spinUp = false;
    addRequirements(launcher);
  }

  public LauncherAngleCmd(LauncherSubsystem launcher, DoubleSupplier setPos, boolean spinUp, boolean isAuto) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcher = launcher;
    this.setPos = setPos;
    this.isAuto = isAuto;
    this.spinUp = spinUp;
    addRequirements(launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if(setPos.getAsDouble() <= 8.55 && setPos.getAsDouble() >= -2) {
    //   if (this.setPos != null)launcher.setAngle(setPos.getAsDouble());
    //   if (this.spinUp) launcher.setReference(1000);
    // }
    // launcher.setAngle(-1.5);
    // launcher.setPivotAbsRef(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // launcher.calcSetAbs();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return isAuto?launcher.getAngle() >= setPos.getAsDouble()-.075 && launcher.getAngle() <= setPos.getAsDouble()+.1:
    //               launcher.getAngle() >= setPos.getAsDouble()-.12 && launcher.getAngle() <= setPos.getAsDouble()+.12;
    // return launcher.getAngle() >= -1.5-.12 && launcher.getAngle() <= -1.5+.12;
    return true;
  }
}
