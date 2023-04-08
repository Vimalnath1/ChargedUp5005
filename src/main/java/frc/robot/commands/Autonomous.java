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
  public Autonomous(DriveTrain drivetrain, Grabber grabber, Climber climber, RobotGyro gyro, ExampleSubsystem example) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new TankDrive(drivetrain,0.2).withTimeout(2),
      new RobotClimb(climber, -1).withTimeout(7.25),
      new LiftArm(grabber, 0.25).withTimeout(2.5),
      new GrabThing(grabber, 0.75).withTimeout(1),
      //new TankDrive(drivetrain, 0.2).withTimeout(0.1),
      //new ExampleCommand(example).withTimeout(0.1),
      /*new LiftArm(grabber, -0.25).withTimeout(2.5),//,
      //new ExampleCommand(example).withTimeout(1),*/
      //new TankDrive(drivetrain, -0.42).withTimeout(2.1)//,
      //new DrivetoDistance(drivetrain,-45)
      new HelpBalance(drivetrain, grabber)
      //new DrivetoDistance(drivetrain, 45)
      //new GyroThing(drivetrain, grabber, climber, gyro, example).withTimeout(10)                                                      //2.9 for charge I think
      //x   new BalanceRobot(drivetrain, gyro).withTimeout(1000)
      //Change and add stuff
    );
  }
}
