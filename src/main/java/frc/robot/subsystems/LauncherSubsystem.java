// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {

  private static LauncherSubsystem instance;

  private CANSparkMax left;
  private CANSparkMax right;
  private CANSparkMax rotation;
  private CANSparkMax belt;

  // private SparkPIDController leftPIDController;
  // private SparkPIDController rightPIDController;
  private SparkPIDController rotationPIDController;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private RelativeEncoder rotationEncoder;

  private double leftSet;
  private double rightSet;

  public LauncherSubsystem() {
    left = new CANSparkMax(9, MotorType.kBrushless);
    right = new CANSparkMax(10, MotorType.kBrushless);
    rotation = new CANSparkMax(7,MotorType.kBrushless);
    belt = new CANSparkMax(6,MotorType.kBrushless);


    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();
    rotation.restoreFactoryDefaults();
    belt.restoreFactoryDefaults();
    
    rotationPIDController = rotation.getPIDController();
  
    // leftPIDController = left.getPIDController();
    // rightPIDController = right.getPIDController();

    leftEncoder = left.getEncoder();
    rightEncoder = right.getEncoder();
    rotationEncoder = rotation.getEncoder();

    // leftPIDController.setP(0);
    // leftPIDController.setI(0);
    // leftPIDController.setD(0);
    // leftPIDController.setIZone(0);
    // leftPIDController.setFF(0);
    // leftPIDController.setOutputRange(-1, 1);
    // leftPIDController.setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal, 0);
    // leftPIDController.setSmartMotionAllowedClosedLoopError(0, 0);

    // rightPIDController.setP(0);
    // rightPIDController.setI(0);
    // rightPIDController.setD(0);
    // rightPIDController.setIZone(0);
    // rightPIDController.setFF(0);
    // rightPIDController.setOutputRange(-1, 1);

    rotationPIDController.setP(0);
    rotationPIDController.setI(0);
    rotationPIDController.setD(0);
    rotationPIDController.setIZone(0);
    rotationPIDController.setFF(0);
    rotationPIDController.setOutputRange(-1, 1);

    leftSet = 0;
    rightSet = 0;
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Left Launcher Velo: ", leftEncoder.getVelocity());
    // SmartDashboard.putNumber("Right Launcher Velo: ", rightEncoder.getVelocity());
    // SmartDashboard.putNumber("Left SetRPM: ", leftSet);
    // SmartDashboard.putNumber("Right SetRPM: ", rightSet);
  }

  // public void setLeftReference(double rpm){
  //   leftPIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity); 
  //   leftSet = rpm;
  // }

  // public void setRightReference(double rpm){
  //   rightPIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity); 
  //   rightSet = rpm;
  // }

  // public void setReference(double rpm){
  //   setLeftReference(rpm);
  //   setRightReference(rpm);
  // }

  public void setAngle(double setpoint) {
    rotationPIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition); 
  }

  public double getAngle() {
    return rotationEncoder.getPosition();
  }

  public void setLeft(double speed){
    left.set(speed);
  }

  public void setRight(double speed){
    right.set(-speed);
  }

  public void runBelt(double speed)
  {
    belt.set(speed);
  }
}
