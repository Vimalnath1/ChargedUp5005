// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class LineUpforCone extends CommandBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  /** Creates a new LineUpforCone. */
   DriveTrain drivetrain;
   double targetVisible;
   double Kpadjust = 0.1; //Subject to change
   double steering_adjust=0.0;
   double distance=0.0;
   double desired_distance; //This will be whatever we decide to be the distance we have to be in order to reach the pole
   double distance_error=0;
   NetworkTableEntry tx;
   NetworkTableEntry ty;
   double x;
   double y;
  public LineUpforCone(DriveTrain subsystem) {
    drivetrain=subsystem;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    targetVisible=NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
    System.out.println(targetVisible);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetVisible=NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
    //if (targetVisible<1.0){       
      //steering_adjust=0.2;
      //drivetrain.turnanddrive(steering_adjust, steering_adjust);
      SmartDashboard.putNumber("tv", targetVisible);
      //}
    //else{
      tx=table.getEntry("tx"); 
      ty=table.getEntry("ty");
      x=tx.getDouble(0.0);
      y=ty.getDouble(0.0);
      SmartDashboard.putNumber("ty", x);
      //System.out.println(x); 
      SmartDashboard.putNumber("tx", y);
      steering_adjust = Kpadjust * x;
      if (x>1.0 && x<-1.0){
        //drivetrain.turnanddrive(steering_adjust, steering_adjust);
      }
      if (x<1.0 && x>-1.0){
        //distance=getDistancefromPole(10,15,32,ty); //Bogus values,change later
        //distance_error=desired_distance-distance;
      }
    drivetrain.drivedistance(distance_error);
  }

  public void Update_Limelight(){
    
      }
  //}

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
