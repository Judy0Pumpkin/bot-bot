// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

// import com.ctre.phoenix.Util;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoBack;
import frc.robot.commands.AutoLeft;
import frc.robot.commands.AutoMiddle;
import frc.robot.commands.AutoRight;
import frc.robot.commands.AutoTakeIn;
import frc.robot.commands.Back;
import frc.robot.commands.Output;
import frc.robot.commands.Stop;
import frc.robot.commands.TakeIn;
import frc.robot.commands.TakeInA;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// import edu.wpi.first.math.controller.HolonomicDriveController;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import frc.robot.Constants.AutoConstants;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Intake m_intake = new Intake();
  private final Arm m_arm = new Arm();
  private final Elevator m_Elevator = new Elevator();
  private final Stop mStop = new Stop(m_intake, m_arm, m_Elevator, m_robotDrive);
  private final PathPlannerTrajectory mblue1to1 = PathPlanner.loadPath("blue1-1", new PathConstraints(1, 1));
  private final PathPlannerTrajectory mblue1to2 = PathPlanner.loadPath("blue1-2", new PathConstraints(1, 1));
  private final PathPlannerTrajectory mblue1to3 = PathPlanner.loadPath("blue1-3", new PathConstraints(1, 1));
  private final PathPlannerTrajectory mblue2to1 = PathPlanner.loadPath("blue2-1", new PathConstraints(1, 1));
  private final PathPlannerTrajectory mblue2to2 = PathPlanner.loadPath("blue2-2", new PathConstraints(1, 1));
  private final PathPlannerTrajectory mblue3to1 = PathPlanner.loadPath("blue 3-1", new PathConstraints(1, 1));
  private final PathPlannerTrajectory mblue3to2 = PathPlanner.loadPath("blue 3-2", new PathConstraints(1, 1));
  private final PathPlannerTrajectory mblue3to3 = PathPlanner.loadPath("blue 3-3", new PathConstraints(1, 1));
  
 
  // private final PathPlannerTrajectory mred1to3 = PathPlanner.loadPath("blue1-3", new PathConstraints(0.5, 1));
  
  
  private final TakeIn mTakeIn =new TakeIn(m_intake, m_arm, m_Elevator);
  private final TakeInA mTakeIn2 = new TakeInA(m_intake, m_arm, m_Elevator);
  private final Output m_OutPut = new Output(m_intake, m_arm, m_Elevator);
  private final Back m_Back = new Back(m_intake, m_arm, m_Elevator);
  private final AutoBack m_AutoBack = new AutoBack(m_intake, m_arm, m_Elevator);  
  private final AutoTakeIn m_AutoTakeIn = new AutoTakeIn(m_intake, m_arm, m_Elevator);
  private final  AutoLeft m_AutoLeft = new AutoLeft(m_intake, m_arm, m_Elevator, m_AutoTakeIn, mTakeIn2, m_AutoBack, m_robotDrive, mblue1to1, mblue1to2, mblue1to3, m_OutPut);
  // AutoRight m_AutoRight = new AutoRight(m_intake, m_arm, m_Elevator, mTakeIn, mTakeIn2, m_Back, m_robotDrive, mblue3to1, mblue3to2, mblue3to3, m_OutPut);
  // AutoMiddle m_AutoMiddle = new AutoMiddle(m_intake, m_arm, m_Elevator, mTakeIn, mTakeIn2, m_Back, m_robotDrive, mblue2to1, mblue2to2, m_OutPut);
  // The driver's controller
  XboxController Joystick = new XboxController(OIConstants.kJoystickPort);
  XboxController driverstation = new XboxController(OIConstants.kDriverstationPort);
  // Joystick m_Joystick = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                  Joystick.getLeftY()*3,
                  Joystick.getLeftX()*3,
                  Joystick.getRightX()*2.5,
                  true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
  
   // new JoystickButton(driverstation, 2).whileTrue(Commands.runOnce(() -> m_robotDrive.drive( 0,0,0,false), m_robotDrive));
   // new JoystickButton(driverstation, 3).whileTrue(Commands.runOnce(() -> m_robotDrive.drive( 0.35,0,0,false), m_robotDrive));
    new JoystickButton(driverstation, 4).whileTrue(Commands.run(m_robotDrive::level));
    // new JoystickButton(driverstation, 5).onTrue(Commands.run(m_robotDrive::zeroyaw));
    // new JoystickButton(driverstation, 6).onTrue(Commands.run(m_robotDrive::leftTarget));
    // new JoystickButton(driverstation, 7).onTrue(Commands.run(m_robotDrive::rightTarget));
    // new JoystickButton(driverstation, 8).whileTrue(Commands.run(m_intake::intakeopen));
    
    new JoystickButton(driverstation,4).onTrue(Commands.runOnce(m_Elevator::level3).andThen(Commands.runOnce(m_arm::armOut))); 
    new JoystickButton(driverstation,1).onTrue(Commands.runOnce(m_Elevator::level2).andThen(Commands.runOnce(m_arm::armOut)));  
    new JoystickButton(driverstation,7).onTrue(Commands.runOnce(m_Elevator::level1).andThen(Commands.runOnce(m_arm::armOut)));  
    // new JoystickButton(Joystick, 4).onTrue(Commands.runOnce(m_robotDrive::zeroyaw, m_robotDrive));
    new JoystickButton(driverstation,3).whileTrue(m_Back);
    ///.onTrue(m_Back); 
    

     
         
  // //   /*------------------------------------------------------------------------------------------------------- */                            


  
   new JoystickButton(driverstation, 5).onTrue(mTakeIn);
   new JoystickButton(driverstation, 6).onTrue(mTakeIn2);
   new JoystickButton(Joystick, 2).onTrue(m_OutPut);


      // new JoystickButton(driverstation, 1).whileTrue(Commands.runOnce(m_arm::armOut)); 
      // new JoystickButton(driverstation, 2).onTrue(Commands.runOnce(m_arm::armIn));    
      // new JoystickButton(driverstation, 3).onTrue(Commands.runOnce(m_arm::tubeForward)); 
      // new JoystickButton(driverstation, 4).onTrue(Commands.runOnce(m_arm::tubeIn)); 
      // new JoystickButton(driverstation, 5).onTrue(Commands.runOnce(m_arm::clawForward));
   
   
      // new JoystickButton(driverstation, 6).onTrue(Commands.runOnce(m_arm::clawrelease));   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() { 
    

return m_AutoLeft;
  }
 }
