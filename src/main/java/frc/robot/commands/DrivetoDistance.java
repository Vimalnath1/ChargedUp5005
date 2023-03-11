// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DrivetoDistance extends CommandBase {
  /** Creates a new DrivetoDistance. */
  DriveTrain drivetrain;
  
  double distance;
  public DrivetoDistance(DriveTrain subsystem,double feet) {
    drivetrain=subsystem;
    distance=feet;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.leftEncoder2.setPosition(0);
    drivetrain.rightEncoder2.setPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drivedistance(distance,drivetrain.leftEncoder2,drivetrain.rightEncoder2);
   //drivetrain.tankdrive(0.2, -0.2);
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
