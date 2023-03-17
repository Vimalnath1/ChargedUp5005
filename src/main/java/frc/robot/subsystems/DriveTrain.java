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
import frc.robot.RobotContainer;

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
  public RelativeEncoder leftEncoder2;
  RelativeEncoder rightEncoder1;
  public RelativeEncoder rightEncoder2;
  PIDController left2pidController;
  PIDController right2pidController;
  double kp=0.05; 
  double ki=0;
  double kd=0;
  double leftposition=0;
  double rightposition=0;
  double creepspeed=0.1;
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
    //leftEncoder2.setPositionConversionFactor((1/8.46)*(6*Math.PI)*(1/12));
    rightEncoder2=rightback.getEncoder();
    //rightEncoder2.setPositionConversionFactor((1/8.46)*(6*Math.PI)*(1/12));
    //left2pidController.setPID(kp, ki, kd);
    //right2pidController.setPID(kp, ki, kd);
    //leftEncoder2.setPosition(0);
    //rightEncoder2.setPosition(0);
  }
  public void turnanddrive(double xAxis, double yAxis,double limit, double creepturn, double creepstraight){
    if (RobotContainer.reverse==true){
      creepspeed=-0.1;
      xAxis=xAxis*-1;
      yAxis=yAxis*-1;
    }
    else{
      creepspeed=0.1;
    }/*
      if (creepstraight==1){
      leftback.set(-creepspeed);
      rightback.set(creepspeed);
      leftfront.set(-creepspeed);
      rightfront.set(creepspeed);
      }
      else{
        leftback.set(creepspeed);
        rightback.set(-creepspeed);
        leftfront.set(creepspeed);
        rightfront.set(-creepspeed);
      }*/
      if (creepturn>0.6){
      leftfront.set(creepspeed);
      leftback.set(creepspeed);
      rightfront.set(creepspeed);
      rightback.set(creepspeed);
      }
      else if (creepturn<-0.6){
        leftfront.set(-creepspeed);
        leftback.set(-creepspeed);
        rightfront.set(-creepspeed);
        rightback.set(-creepspeed);
      }
    if (creepturn<0.6 && creepturn>-0.6){
    if (xAxis<-0.3 || xAxis>0.3 ){
      /*if (xAxis>0.3){
        xAxis=0.3;
      }
      if (xAxis<-0.3){
        xAxis=-0.3;
      }
      /*leftfront.set(xAxis);
      leftback.set(xAxis);
      rightfront.set(xAxis);
      rightback.set(xAxis);*/
      if (xAxis<-0.3){
        if (xAxis<-0.3){
          xAxis=-0.3;
        }
        if (yAxis<0){
          if (yAxis<-0.3){
            yAxis=-0.3;
          }
          tankdrive(xAxis+0.05, xAxis);
        }
        else{
          if (yAxis>0.3){
            yAxis=0.3;
          }
          tankdrive(xAxis+0.05, -xAxis);
        }
      }
      if (xAxis>0.3){
        if (xAxis>0.3){
          xAxis=0.3;
        }
        if (yAxis<0){
          if (yAxis<-0.3){
            yAxis=-0.3;
          }
          tankdrive(xAxis, xAxis+0.05);
        }
        else{
          if (yAxis>0.3){
            yAxis=0.3;
          }
          tankdrive(-xAxis, xAxis-0.05);
        }
      }
    }
    else{
      if (yAxis>limit){
        yAxis=limit;
      }
      if (yAxis<-limit){
        yAxis=-limit;
      }
      if (yAxis>0.1 || yAxis<-0.1){
      leftback.set(-yAxis);
      rightback.set(yAxis);
      leftfront.set(-yAxis);
      rightfront.set(yAxis);
      }
      else{
      leftback.set(0);
      rightback.set(0);
      leftfront.set(0);
      rightfront.set(0);
      }
    }
    /*else{
      /*if (xAxis<-0.3 || xAxis>0.3 && yAxis>0.15 || yAxis<-0.15){
        if (xAxis<-0.5 && yAxis<-0.5){
          tankdrive(0,yAxis);
        }
        if (xAxis>0.5 && yAxis<-0.5){
          tankdrive(xAxis, 0);
        }
        if (xAxis<-0.5 && yAxis>0.5){           Test this stuff
          tankdrive(0,yAxis);
        }
        if (xAxis>0.5 && yAxis>0.5){
          tankdrive(-xAxis,0);
        }
      }*/ 
      
    }
  //}
  }
  public void tankdrive(double leftspeed,double rightspeed){
    leftfront.set(leftspeed);
    leftback.set(leftspeed);
    rightfront.set(rightspeed);
    rightback.set(rightspeed);
  }
  public void turn(double xAxis){
    leftfront.set(xAxis);
    leftback.set(xAxis);
    rightfront.set(xAxis);
    rightback.set(xAxis);
  }
  public void drivedistance(double distance,RelativeEncoder encoder1, RelativeEncoder encoder2){
    tankdrive(0, 0);
    //leftEncoder2.setPosition(0);
    //rightEncoder2.setPosition(0);
    double leftposition=0;
    double rightposition=0;
    encoder1.setPositionConversionFactor((1/4096)*(1/8.46)*(6*Math.PI)*(1/12));
    encoder2.setPositionConversionFactor((1/4096)*(1/8.46)*(6*Math.PI)*(1/12));
    //SmartDashboard.putNumber("Left Encoder Position",leftposition);
    //double leftspeed=left2pidController.calculate(distance);
    //double rightspeed=right2pidController.calculate(distance);
    while (leftposition<distance && rightposition>-distance){
      double leftspeed=0;
      double rightspeed=0;
      leftspeed=left2pidController.calculate(distance-leftposition);
      rightspeed=right2pidController.calculate(distance+rightposition);
     
      leftposition=encoder1.getPosition();
      rightposition=encoder2.getPosition();
      System.out.println(rightposition);
      SmartDashboard.putNumber("Left Encoder Position",leftposition);
      SmartDashboard.putNumber("Left Speed",leftspeed);
      SmartDashboard.putNumber("Right Speed", rightspeed);
      tankdrive(0.2, -0.2);
    }
    if (leftposition>distance && rightposition<-distance){
      tankdrive(0, 0);
      
    }

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
