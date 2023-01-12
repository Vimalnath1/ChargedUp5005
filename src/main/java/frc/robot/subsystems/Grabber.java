// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {
  Spark armmotor;
  Spark clawmotor;
  /** Creates a new Grabber. */
  public Grabber() {
    armmotor= new Spark(Constants.armmotornumber);
    clawmotor=new Spark(Constants.clawmotornumber);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void liftarm(double speed){
    armmotor.set(speed);
  }
  public void grab(double speed){
    clawmotor.set(speed);
  }
}
