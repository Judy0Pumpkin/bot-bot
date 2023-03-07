// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Back extends SequentialCommandGroup {
  /** Creates a new Back. */
  public Back(Intake m_intake, Arm m_arm, Elevator m_Elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(()->m_arm.armIn(), m_arm));
    addCommands(new InstantCommand(()->m_arm.tubeForward(), m_arm));
    addCommands(new InstantCommand(()->m_Elevator.back(), m_Elevator).withTimeout(2));

    addRequirements(m_arm, m_Elevator, m_intake);
  }
}
