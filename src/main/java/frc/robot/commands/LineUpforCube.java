// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry; 
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class LineUpforCube extends CommandBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  /** Creates a new LineUpforCone. */
   DriveTrain drivetrain;
   PIDController pidController;
   double targetVisible;
   double Kpadjust = 0.02; //Subject to change
   double Kiadjust=0.0075;
   double Kdadjust=0.01;
   double steering_adjust=0.0;
   double distance=0.0;
   double desired_distance; //This will be whatever we decide to be the distance we have to be in order to reach the pole
   double distance_error=0;
   NetworkTableEntry tx;
   NetworkTableEntry ty;
   double x;
   double y;
   //double time;
   //double dt;
   //double errorsum;
  public LineUpforCube(DriveTrain subsystem) {
    drivetrain=subsystem;
    addRequirements(drivetrain);
    pidController=new PIDController(Kpadjust, Kiadjust, Kdadjust);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //time=Timer.getFPGATimestamp();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
    Update_Limelight();
  }

  public void Update_Limelight(){
    targetVisible=NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
    if (targetVisible<1.0){       
      //steering_adjust=0.31;
      //drivetrain.turnanddrive(steering_adjust, steering_adjust);
      }
    else{
      
      //dt=Timer.getFPGATimestamp()-time;
      tx=table.getEntry("tx");
      ty=table.getEntry("ty");
      x=tx.getDouble(0.0);
      y=ty.getDouble(0.0);
      SmartDashboard.putNumber("ty", y);
      //System.out.println(x); 
      SmartDashboard.putNumber("tx", x); 
      //errorsum=x*dt;
      //steering_adjust = Kpadjust * x+ Kiadjust*errorsum;
      pidController.setIntegratorRange(-1, 1);
      steering_adjust=pidController.calculate(x);
      //SmartDashboard.putNumber("Ki", Kiadjust*errorsum);
      SmartDashboard.putNumber("Steering Adjust", steering_adjust);
      if (x>1.0 || x<-1.0){
        drivetrain.turn(-steering_adjust);
      }
      if (x<1.0 || x>-1.0){
        distance=getDistancefromPole(9,33.31,19,y); //Bogus values,change later
        SmartDashboard.putNumber("distance", distance);
        distance_error=desired_distance-distance;
      }
      }
  }

  public double getDistancefromPole(double heightOfLimelight, double heightOfGoal, double angleOfLimelight,double yOffset){
    yOffset+=angleOfLimelight;
    yOffset=Math.toRadians(yOffset);
    return (heightOfGoal-heightOfLimelight)/Math.tan(yOffset);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.turnanddrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
