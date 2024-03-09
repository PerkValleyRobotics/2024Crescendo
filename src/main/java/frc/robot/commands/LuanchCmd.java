// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LuanchCmd extends SequentialCommandGroup {
  /** Creates a new LuanchCmd. */
  public LuanchCmd(LauncherSubsystem launcher, ConveyorSubsystem conveyor, DoubleSupplier angle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new LauncherAngleCmd(launcher, angle, true, true).withTimeout(1), 
                new ParallelCommandGroup(
                  new RunLauncherCmd(launcher, () -> 0.9625).withTimeout(2),
                  new RunBeltCmd(conveyor, -.75).withTimeout(2)),
                new LauncherAngleCmd(launcher, () -> 0, true));
  }
}
