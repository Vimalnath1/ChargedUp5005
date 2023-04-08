// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class BalanceRobot extends CommandBase {
  private final DriveTrain drivetrain;
  private final RobotGyro gyro;
  private double angle;
  PIDController pidController;
  double kp=0.01;
  double ki=0;
  double kd=0;
  double speed=0;
  /** Creates a new BalanceRobot. */
  public BalanceRobot(DriveTrain subsystem, RobotGyro gyrosubsystem) {
    drivetrain=subsystem;
    gyro=gyrosubsystem;
    pidController=new PIDController(kp, ki, kd);
    addRequirements(drivetrain,gyro);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override 
  public void initialize() {
  
    gyro.ResetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle=gyro.getrobotAngle();
    speed=pidController.calculate(angle);
    System.out.println(angle);
    SmartDashboard.putNumber("Angle", angle);
    if (angle<-5){
      drivetrain.tankdrive(-0.15, 0.15);
      //drivetrain.tankdrive(speed,-speed);
    }
    else if (angle>5){
      drivetrain.tankdrive(0.15, -0.15); 
      //drivetrain.tankdrive(speed,-speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.turnanddrive(0, 0,0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
