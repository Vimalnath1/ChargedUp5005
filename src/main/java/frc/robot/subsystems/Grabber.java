// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {
  CANSparkMax armmotor;
  Spark clawmotor1;
  Spark clawmotor2;
  Spark rotatingclaw;
  DigitalInput limitswitchtop;
  DigitalInput limitswitchbottom;
  /** Creates a new Grabber. */
  public Grabber() {
    armmotor= new CANSparkMax(5,MotorType.kBrushless);
    clawmotor1=new Spark(6);
    clawmotor2=new Spark(8);
    rotatingclaw=new Spark(7);
    limitswitchtop=new DigitalInput(7);
    limitswitchbottom=new DigitalInput(8);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void liftarm(double speed){
    armmotor.set(speed);
  }
  public void grab(double speed){
    clawmotor1.set(speed);
    clawmotor2.set(-speed);
  }
  public void rotateclaw(double speed){
    rotatingclaw.set(speed);
    /*
    if (limitswitchtop.get()==false && limitswitchbottom.get()==false){    
      rotatingclaw.set(speed);
    }
    else if (limitswitchtop.get()==true){
      if (speed<0){
        speed=0;
      }
      rotatingclaw.set(speed);
      SmartDashboard.putBoolean("Claw Top Limit", limitswitchtop.get());
    }
    else if (limitswitchbottom.get()==true){
      if (speed>0){
        speed=0;
      }
      rotatingclaw.set(speed);
      SmartDashboard.putBoolean("Bottom Top Limit", limitswitchbottom.get());
      System.out.println(limitswitchbottom.get());
    }*/
  }
}
