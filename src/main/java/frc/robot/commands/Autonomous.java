// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autonomous extends SequentialCommandGroup {
  /** Creates a new Autonomous. */
  public Autonomous(DriveTrain drivetrain, Grabber grabber, Climber climber, RobotGyro gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new TankDrive(drivetrain,0.2).withTimeout(2),
      new LiftArm(grabber, 0.25).withTimeout(2.5),
      new ReleaseGrabber(grabber),
      new LiftArm(grabber, -0.25).withTimeout(1),
      new TankDrive(drivetrain, -0.2).withTimeout(2)

      //Change and add stuff
    );
  }
}
