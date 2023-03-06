// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
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
public class Stop extends SequentialCommandGroup {
  Timer mTimer = new Timer();
  /** Creates a new OutPut. */
  public Stop(Intake m_intake, Arm m_arm, Elevator m_Elevator, DriveSubsystem m_DriveSubsystem) {

    addCommands(new InstantCommand(()->m_Elevator.elevatorstop(),m_Elevator));
    
    addCommands(new InstantCommand(()->m_intake.neoStop(),m_intake));
    
    addCommands(new InstantCommand(()->m_DriveSubsystem.drive(0,0,0,true)));
    addRequirements(m_arm, m_Elevator, m_intake); 
  
  } 

  
}
