// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.Compressor;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax intake;
  DoubleSolenoid intakeSolenoid;
  boolean toggle;
  //Compressor comp = new Compressor(PneumaticsModuleType.CTREPCM);

  public IntakeSubsystem() {
    intake = new CANSparkMax(Constants.KIntakeMotorID, CANSparkLowLevel.MotorType.kBrushless);

    toggle = false;

    intakeSolenoid = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 1, 0);
  }

  @Override
  public void periodic() {
    // System.out.print("Whoh you have an intake subsystem!");
    // This method will be called once per scheduler run
  }
  public void toggleIntake()
  {
    if (toggle) intakeSolenoid.set(Value.kReverse);
    else intakeSolenoid.set(Value.kForward);
    toggle = !toggle;
  }

  public void run(double speed) {
    intake.set(speed);
  }
}
