// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DefaultDrive extends CommandBase {
  private final DriveTrain drivetrain;
  public final DoubleSupplier leftvalue;
  public final DoubleSupplier rightvalue;
  public final DoubleSupplier turnvalue;
  public final DoubleSupplier straightvalue;
  /** Creates a new DefaultDrive. */
  public DefaultDrive(DriveTrain subsystem,DoubleSupplier left,DoubleSupplier right,DoubleSupplier creepturn, DoubleSupplier creepstraight) {
    drivetrain=subsystem;
    leftvalue=left;
    rightvalue=right;
    turnvalue=creepturn;
    straightvalue=creepstraight;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.turnanddrive(leftvalue.getAsDouble(), rightvalue.getAsDouble(),0.3,turnvalue.getAsDouble(),straightvalue.getAsDouble());
    //SmartDashboard.putNumber("Left Front Encoder", DriveTrain.leftEncoder1.getPosition());
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
