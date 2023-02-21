// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
// import com.ctre.phoenix.Util;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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
  private final Intake intake = new Intake();
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
                  Joystick.getLeftY(),
                  Joystick.getLeftX(),
                  Joystick.getRightX()*2,
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
    new JoystickButton(driverstation, 5).onTrue(Commands.run(m_robotDrive::zeroyaw));
    new JoystickButton(driverstation, 6).onTrue(Commands.run(m_robotDrive::leftTarget));
    new JoystickButton(driverstation, 7).onTrue(Commands.run(m_robotDrive::rightTarget));
    new JoystickButton(driverstation, 8).whileTrue(Commands.run(intake::intakeopen));
    new JoystickButton(driverstation, 8).whileFalse(Commands.run(intake::intakeoff));                                     

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() { 

    String trajectoryJSON = "output/turn.wpilib.json";
    Trajectory trajectory = new Trajectory();
    
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(trajectory.getInitialPose());

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
          trajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            m_robotDrive.controller,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(trajectory.getInitialPose());

    //Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));

  }
 }
