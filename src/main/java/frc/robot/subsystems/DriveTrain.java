// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  CANSparkMax leftfront;
  CANSparkMax rightfront;
  CANSparkMax leftback;
  CANSparkMax rightback;
  public static RelativeEncoder leftEncoder1;
  RelativeEncoder leftEncoder2;
  RelativeEncoder rightEncoder1;
  RelativeEncoder rightEncoder2;
  SparkMaxPIDController leftpidController;
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftfront=new CANSparkMax(Constants.leftfrontnumber,MotorType.kBrushless); //1
    rightfront=new CANSparkMax(Constants.rightfrontnumber,MotorType.kBrushless); //3
    leftback=new CANSparkMax(Constants.leftbacknumber,MotorType.kBrushless);//4
    rightback=new CANSparkMax(Constants.rightbacknumber,MotorType.kBrushless);//2
    leftEncoder1=leftfront.getEncoder();
    leftEncoder1.setPositionConversionFactor((8.46*(6*Math.PI)*(1/12)));
  }
  public void turnanddrive(double xAxis, double yAxis){
    if (xAxis<-0.3 || xAxis>0.3){
      leftfront.set(xAxis);
      leftback.set(xAxis);
      rightfront.set(xAxis);
      rightback.set(xAxis);
    }
    else{
      leftback.set(-yAxis);
      rightback.set(yAxis);
      leftfront.set(yAxis);
      rightfront.set(-yAxis);

    }
  
  }
  public void drivedistance(double distance){
    double leftposition=leftEncoder1.getPosition();
    SmartDashboard.putNumber("Left Encoder Position",leftposition);
    if (leftposition<distance){
      turnanddrive(0.5, -0.5);
    }
    else{
      turnanddrive(0, 0);
    }

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
