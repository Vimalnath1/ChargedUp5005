// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class BalanceRobot extends CommandBase {
  private final DriveTrain drivetrain;
  private final RobotGyro gyro;
  private double angle;
  /** Creates a new BalanceRobot. */
  public BalanceRobot(DriveTrain subsystem, RobotGyro gyrosubsystem) {
    drivetrain=subsystem;
    gyro=gyrosubsystem;
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
    /*if (angle<0){
      drivetrain.turnanddrive(0.5, 0.5);
      angle=gyro.getrobotAngle();
    }
    else if (angle>0){
      drivetrain.turnanddrive(-0.5, -0.5);
      angle=gyro.getrobotAngle();
    }*/
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
