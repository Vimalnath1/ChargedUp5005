// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  Spark leftfront;
  Spark rightfront;
  Spark leftback;
  Spark rightback;
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftfront=new Spark(Constants.leftfrontnumber);
    rightfront=new Spark(Constants.rightfrontnumber);
    leftback=new Spark(Constants.leftbacknumber);
    rightback=new Spark(Constants.rightbacknumber);
  }
  public void turnanddrive(double xAxis, double yAxis){
    if (xAxis<-0.3 || xAxis>0.3){
      leftfront.set(xAxis);
      leftback.set(xAxis);
      rightfront.set(xAxis);
      rightback.set(xAxis);
    }
    else{
      leftfront.set(-yAxis);
      leftback.set(yAxis);
      rightfront.set(-yAxis);
      rightback.set(yAxis);

    }

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
