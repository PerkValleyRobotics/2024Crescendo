// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax intake;
  Solenoid solenoid;
  Solenoid solenoid2;
  public IntakeSubsystem() {
    intake = new CANSparkMax(Constants.KIntakeMotorID, CANSparkLowLevel.MotorType.kBrushless);
    solenoid = new Solenoid(1, PneumaticsModuleType.CTREPCM, 0);
    solenoid2 = new Solenoid(1, PneumaticsModuleType.CTREPCM, 1);
  }

  @Override
  public void periodic() {
    //System.out.print("Whoh you have an intake subsystem!");
    // This method will be called once per scheduler run
  }
  public void toggleIntake()
  {
    solenoid.toggle();
    solenoid2.toggle();
  }
  public void run(double speed) {
    intake.set(speed);
  }
}
