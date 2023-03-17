// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  CANSparkMax climbermotor;
  DigitalInput toplimitswitch;
  DigitalInput bottomlimitswitch;
  //RelativeEncoder climberEncoder;
  /** Creates a new Climber. */
  public Climber() {
    climbermotor=new CANSparkMax(Constants.climbermotornumber,MotorType.kBrushless);
    toplimitswitch=new DigitalInput(0);
    bottomlimitswitch=new DigitalInput(1);
    //climberEncoder=climbermotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per sc++heduler run
  }

  public void climb(double speed){
    if (toplimitswitch.get()==false && bottomlimitswitch.get()==false){
    climbermotor.set(speed);
    System.out.println(toplimitswitch.get());
    }
    else if (toplimitswitch.get()==true){
      if (speed<0){
        speed=0;
      }
      climbermotor.set(speed);
    }
    else{
      if (speed>0){
        speed=0;
      }
      climbermotor.set(speed);
    }
  }
}
