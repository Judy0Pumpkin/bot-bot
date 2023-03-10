// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoLeft extends SequentialCommandGroup {
  /** Creates a new AutoLeft. */
  public AutoLeft(Intake m_intake, Arm m_arm, Elevator m_Elevator, AutoTakeIn mTakeIn, TakeInA mTakeIn2, AutoBack m_Back, DriveSubsystem m_robotDrive, PathPlannerTrajectory mblue1to1, PathPlannerTrajectory mblue1to2,PathPlannerTrajectory mblue1to3, Output m_Output ) {

    addCommands(mTakeIn);
    addCommands(new InstantCommand(()->m_intake.mini(), m_intake));
    addCommands(new RunCommand(()->m_Elevator.level3(), m_Elevator).withTimeout(2));
    addCommands((m_Output));
    addCommands(m_Back.withTimeout(2));
    addCommands( new WaitCommand(1.2).andThen(m_robotDrive.followTrajectoryCommand(mblue1to2, false).withTimeout(3)));
  }
}
