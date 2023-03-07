// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  Timer m_Timer = new Timer(); 
  CANSparkMax intakeleft = new CANSparkMax( IntakeConstants.intakeleftport, MotorType.kBrushless);
  CANSparkMax intakeright = new CANSparkMax( IntakeConstants.intakerightport, MotorType.kBrushless);


  /** Creates a new Intake. */
  public Intake() {

    intakeleft.restoreFactoryDefaults();
    intakeright.restoreFactoryDefaults();
    intakeleft.setInverted(true);
    intakeright.setInverted(true);
    intakeleft.setIdleMode(IdleMode.kBrake);
    intakeright.setIdleMode(IdleMode.kBrake);
  }
  
 public void intakeStart(){
     m_Timer.start();
    intakeleft.set(0.1);
    intakeright.set(-0.1);
    }

 public void neoStop(){
    intakeleft.set(0);
    intakeright.set(0);

    }
 public void mini(){
  intakeleft.set(0.05);
  intakeright.set(-0.05);
  
 }

  // public void intakeStart(){
  //    m_Timer.start();
  //   intakeleft.set(0.4);
  
  //   if (m_Timer.get() > 3.0) {
  //     intakeleft.set(0.0);
  //       m_Timer.stop();
  //       m_Timer.reset();
  //   }


  // }

  public void intakeStop(){
    intakeleft.set(0);
    intakeright.set(0);
  }







  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
