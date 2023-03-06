// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRight extends SequentialCommandGroup {
  /** Creates a new AutoRight. */
  public AutoRight(Intake m_intake, Arm m_arm, Elevator m_Elevator, TakeIn mTakeIn, TakeInA mTakeIn2, Back m_Back, DriveSubsystem m_robotDrive, PathPlannerTrajectory mblue3to1, PathPlannerTrajectory mblue3to2,PathPlannerTrajectory mblue3to3, Output m_Output ) {


    addCommands(mTakeIn2);
    addCommands( m_robotDrive.followTrajectoryCommand(mblue3to1, true));
    addCommands(mTakeIn);
    addCommands(new WaitCommand(2).andThen(()->m_Elevator.level3(), m_Elevator));
    addCommands(new WaitCommand(3).andThen(m_Output));
    addCommands(m_Back);
    addCommands(m_robotDrive.followTrajectoryCommand(mblue3to2, false));
  }
}
