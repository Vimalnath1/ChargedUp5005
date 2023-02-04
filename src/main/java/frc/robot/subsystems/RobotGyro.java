// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotGyro extends SubsystemBase {
  ADIS16470_IMU gyro=new ADIS16470_IMU();
  /** Creates a new RobotGyro. */
  public RobotGyro() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public double getrobotAngle(){
    return gyro.getYComplementaryAngle();
  }

  public void ResetGyro(){
    gyro.reset();
  }
}
