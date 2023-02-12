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
  Spark leftfrontspark;
  Spark rightfrontspark;
  Spark leftbackspark;
  Spark rightbackspark;
  public static RelativeEncoder leftEncoder1;
  RelativeEncoder leftEncoder2;
  RelativeEncoder rightEncoder1;
  RelativeEncoder rightEncoder2;
  PIDController left2pidController;
  PIDController right2pidController;
  double kp=0.1;
  double ki=0;
  double kd=0;
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    //leftfront=new CANSparkMax(Constants.leftfrontnumber,MotorType.kBrushless); //1
    //rightfront=new CANSparkMax(Constants.rightfrontnumber,MotorType.kBrushless); //3
    leftback=new CANSparkMax(Constants.leftbacknumber,MotorType.kBrushless);//4
    rightback=new CANSparkMax(Constants.rightbacknumber,MotorType.kBrushless);//1
    /*leftfrontspark=new Spark(0);
    rightfrontspark=new Spark(2);
    leftbackspark=new Spark(3);
    rightbackspark=new Spark(1);*/
    leftEncoder2=leftback.getEncoder();
    leftEncoder2.setPositionConversionFactor((8.46*(6*Math.PI)*(1/12)));
    rightEncoder2=leftback.getEncoder();
    rightEncoder2.setPositionConversionFactor((8.46*(6*Math.PI)*(1/12)));
    left2pidController.setPID(kp, ki, kd);
    right2pidController.setPID(kp, ki, kd);
  }
  public void turnanddrive(double xAxis, double yAxis){
    if (xAxis<-0.3 || xAxis>0.3){
      //leftfront.set(xAxis);
      leftback.set(xAxis);
      //rightfront.set(xAxis);
      rightback.set(xAxis);
      /*leftfrontspark.set(xAxis);
      leftbackspark.set(xAxis);
      rightfrontspark.set(xAxis);
      rightbackspark.set(xAxis);*/
    }
    else{
      leftback.set(-yAxis);
      rightback.set(yAxis);
      /*leftfront.set(yAxis);
      rightfront.set(-yAxis);
      leftbackspark.set(yAxis);
      rightbackspark.set(-yAxis);
      leftfrontspark.set(yAxis);
      rightfrontspark.set(-yAxis);*/

    }
  
  }
  public void turn(double xAxis){
    /*leftfrontspark.set(xAxis);
    leftbackspark.set(xAxis);
    rightfrontspark.set(xAxis);
    rightbackspark.set(xAxis);*/
    //leftfront.set(xAxis);
    leftback.set(xAxis);
    //rightfront.set(xAxis);
    rightback.set(xAxis);
  }
  public void drivedistance(double distance){
    double leftposition=leftEncoder2.getPosition();
    double rightposition=rightEncoder2.getPosition();
    SmartDashboard.putNumber("Left Encoder Position",leftposition);
    if (leftposition<distance && rightposition<distance){
      double leftspeed=left2pidController.calculate(distance-leftposition);
      double rightspeed=right2pidController.calculate(distance-rightposition);
      turnanddrive(leftspeed, -rightspeed);
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
