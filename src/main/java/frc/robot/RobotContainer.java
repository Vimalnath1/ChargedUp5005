// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain m_DriveTrain= new DriveTrain();
  private final Grabber m_grabber= new Grabber();
  private final Climber m_climber= new Climber();
  private final RobotGyro m_gyro=new RobotGyro();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final Autonomous m_gameAutonomous=new Autonomous(m_DriveTrain, m_grabber, m_climber, m_gyro,m_exampleSubsystem); 
  public static Joystick controller= new Joystick(0);
  public static boolean reverse=false;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //if (reverse==true){
      //m_DriveTrain.setDefaultCommand(new DefaultDrive(m_DriveTrain, ()->-controller.getRawAxis(0), ()->-controller.getRawAxis(1),()->-controller.getRawAxis(2),()->-controller.getRawAxis(3)));
    //}
    //else{
      m_DriveTrain.setDefaultCommand(new DefaultDrive(m_DriveTrain, ()->controller.getRawAxis(0), ()->controller.getRawAxis(1),()->controller.getRawAxis(2),()->controller.getRawAxis(3)));
    //}
    //m_DriveTrain.setDefaultCommand(new DefaultDrive(m_DriveTrain, ()->controller.getRawAxis(0), ()->controller.getRawAxis(1),()->controller.getRawAxis(2),()->controller.getRawAxis(3)));
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(controller, 5).whileTrue(new LiftArm(m_grabber,-0.25));
    new JoystickButton(controller, 3).whileTrue(new LiftArm(m_grabber,0.25));
    new JoystickButton(controller, 1).whileTrue(new GrabThing(m_grabber,-0.75));
    new JoystickButton(controller, 12).whileTrue(new GrabThing(m_grabber,0.75));
    new JoystickButton(controller, 7).whileTrue(new RobotClimb(m_climber, -1));
    new JoystickButton(controller, 9).whileTrue(new RobotClimb(m_climber, 1));
    //new JoystickButton(controller, 10).whileTrue(new BalanceRobot(m_DriveTrain, m_gyro));
    new JoystickButton(controller, 10).whileTrue(new DrivetoDistance(m_DriveTrain, 39));
    /*new JoystickButton(controller, 6).onTrue(new OtherRotateClaw(m_grabber, -0.5,()->1.25));
    new JoystickButton(controller, 4).onTrue(new OtherRotateClaw(m_grabber, 0.5,()->1.5));*/
    new JoystickButton(controller, 4).whileTrue(new RotateClaw(m_grabber, 0.5));
    new JoystickButton(controller, 6).whileTrue(new RotateClaw(m_grabber, -0.5));
    new JoystickButton(controller, 2).whileTrue(new LineUpforCube(m_DriveTrain));
    //new JoystickButton(controller, 11).whileTrue(new DropArmandClimb(m_climber,m_grabber, -0.2, 0.2));
    new JoystickButton(controller, 8).whileTrue(new TurboButton(m_DriveTrain));
    new JoystickButton(controller,11).whileTrue(new Reverse());
    
      
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_gameAutonomous;
  }
}
