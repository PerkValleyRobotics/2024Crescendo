// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {
  /** Creates a new ConveyorSubsystem. */
  private static ConveyorSubsystem instance;
  
  private CANSparkMax belt;

  public ConveyorSubsystem() {
    belt = new CANSparkMax(Constants.KLauncherBeltMotorID,MotorType.kBrushless);
    
    belt.restoreFactoryDefaults();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runBelt(double speed)
  {
    belt.set(speed);
  }

  public static  ConveyorSubsystem getInstance() {
    if (instance == null){
      instance = new ConveyorSubsystem();
      return instance;
    } else {
      return instance;
    }
  }
}
