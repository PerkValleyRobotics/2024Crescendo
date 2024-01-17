// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax intake;

  public Intake() {
    intake = new CANSparkMax(Constants.KIntakeMotorID, CANSparkLowLevel.MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    System.out.print("Whoh you have an intake subsystem!");
    // This method will be called once per scheduler run
  }

  public void run(double speed) {
    intake.set(speed);
  }
}
