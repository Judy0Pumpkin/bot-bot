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
public class Output extends SequentialCommandGroup {
  Timer mTimer = new Timer();
  /** Creates a new OutPut. */
  public Output(Intake m_intake, Arm m_arm, Elevator m_Elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(()->m_intake.intakeStop(),m_intake));
    addCommands(new InstantCommand(()->m_arm.tubeIn(), m_arm));
    addCommands(new InstantCommand(()->m_arm.clawrelease(), m_arm));
    // addCommands(new WaitCommand(5).andThen(()->m_intake.intakeshoot(), m_intake));
    // addCommands(new WaitCommand(1).andThen(()->m_intake.intakeStop(), m_intake));
    addCommands(new WaitCommand(2).andThen(()->m_arm.armIn(), m_arm));
   } 

  
}
