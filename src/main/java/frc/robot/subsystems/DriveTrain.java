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
  double kp=0.01; 
  double ki=0;
  double kd=0;
  double leftposition=0;
  double rightposition=0;
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftfront=new CANSparkMax(Constants.leftfrontnumber,MotorType.kBrushless); //1
    rightfront=new CANSparkMax(Constants.rightfrontnumber,MotorType.kBrushless); //8
    leftback=new CANSparkMax(Constants.leftbacknumber,MotorType.kBrushless);//4
    rightback=new CANSparkMax(Constants.rightbacknumber,MotorType.kBrushless);//2
    left2pidController= new PIDController(kp, ki, kd);
    right2pidController= new PIDController(kp, ki, kd);
    leftfrontspark=new Spark(0);
    rightfrontspark=new Spark(2);
    leftbackspark=new Spark(3);
    rightbackspark=new Spark(1);
    leftEncoder2=leftback.getEncoder();
    leftEncoder2.setPositionConversionFactor((1/4096)*(1/8.46)*(6*Math.PI)*(1/12));
    rightEncoder2=rightback.getEncoder();
    rightEncoder2.setPositionConversionFactor((1/4096)*(1/8.46)*(6*Math.PI)*(1/12));
    //left2pidController.setPID(kp, ki, kd);
    //right2pidController.setPID(kp, ki, kd);
  }
  public void turnanddrive(double xAxis, double yAxis){
    if (xAxis<-0.5 || xAxis>0.5){
      //leftfront.set(xAxis);
      leftback.set(xAxis);
      //rightfront.set(xAxis);
      rightback.set(xAxis);
      leftfrontspark.set(xAxis);
      leftbackspark.set(xAxis);
      rightfrontspark.set(xAxis);
      rightbackspark.set(xAxis);
    }
    else{
      leftback.set(-yAxis);
      rightback.set(yAxis);
      /*leftfront.set(yAxis);
      rightfront.set(-yAxis);*/
      leftbackspark.set(yAxis);
      rightbackspark.set(-yAxis);
      leftfrontspark.set(yAxis);
      rightfrontspark.set(-yAxis);

    }
  
  }
  public void tankdrive(double leftspeed,double rightspeed){
    //leftfront.set(xAxis);
    leftback.set(leftspeed);
    //rightfront.set(xAxis);
    rightback.set(rightspeed);
  }
  public void turn(double xAxis){
    leftfrontspark.set(xAxis);
    leftbackspark.set(xAxis);
    rightfrontspark.set(xAxis);
    rightbackspark.set(xAxis);
    //leftfront.set(xAxis);
    leftback.set(xAxis);
    //rightfront.set(xAxis);
    rightback.set(xAxis);
  }
  public void drivedistance(double distance){
    tankdrive(0, 0);
    leftposition=0;
    rightposition=0;
    leftEncoder2.setPosition(0);
    rightEncoder2.setPosition(0);
    //SmartDashboard.putNumber("Left Encoder Position",leftposition);
    //double leftspeed=left2pidController.calculate(distance);
    //double rightspeed=right2pidController.calculate(distance);
    if (leftposition<distance && rightposition<distance){
      /*leftspeed=left2pidController.calculate(distance-leftposition);
      rightspeed=right2pidController.calculate(distance-rightposition);*/
      leftposition=leftEncoder2.getPosition();
      rightposition=rightEncoder2.getPosition();
      System.out.println(rightposition);
      SmartDashboard.putNumber("Left Encoder Position",leftposition);
      tankdrive(0.2, -0.2);
    }
    else{
      tankdrive(0, 0);
      
    }

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
