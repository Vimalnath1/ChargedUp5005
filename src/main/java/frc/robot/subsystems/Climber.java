// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  CANSparkMax climbermotor;
  RelativeEncoder climberEncoder;
  /** Creates a new Climber. */
  public Climber() {
    climbermotor=new CANSparkMax(Constants.climbermotornumber,MotorType.kBrushless);
    climberEncoder=climbermotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per sc++heduler run
  }

  public void climb(double speed){
    climbermotor.set(speed);
  }
}
