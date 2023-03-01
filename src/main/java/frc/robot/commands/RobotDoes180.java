// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.RobotGyro;

public class RobotDoes180 extends CommandBase {
  DriveTrain drivetrain;
  RobotGyro gyro;
  double yawangle;
  double target=180;
  double angleoffset;
  double kp=0.01;
  double ki;
  double kd;
  double speed;
  PIDController pidController;
  /** Creates a new RobotDoes180. */
  public RobotDoes180(DriveTrain subsystem, RobotGyro gyrosubsystem) {
    drivetrain=subsystem;
    gyro=gyrosubsystem;
    pidController= new PIDController(kp, ki, kd);
    addRequirements(drivetrain,gyro);
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    yawangle=gyro.getyaw();
    angleoffset=180-yawangle;
    if (angleoffset>5 || angleoffset<-5){
      speed=pidController.calculate(angleoffset);
      drivetrain.tankdrive(speed,speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
