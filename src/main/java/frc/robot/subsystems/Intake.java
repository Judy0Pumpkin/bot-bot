// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  
 CANSparkMax intakeleft = new CANSparkMax( IntakeConstants.intakeleftport, MotorType.kBrushless);
 CANSparkMax intakeright = new CANSparkMax( IntakeConstants.intakerightport, MotorType.kBrushless);
 private final DoubleSolenoid arm = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
 protected static Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
 private boolean out = false;

  /** Creates a new Intake. */
  public Intake() {

    intakeleft.restoreFactoryDefaults();
    intakeright.restoreFactoryDefaults();
    intakeleft.setInverted(false);
    intakeright.setInverted(true);
    compressor.enableDigital();
  }
  
  public void intakeopen(){
    intakeleft.set(0.1);
    intakeright.set(0.1);
  }

  public void intakeoff(){
    intakeleft.set(0);
    intakeright.set(0);
  }

  public void act() {
    if(out) {
      arm.set(DoubleSolenoid.Value.kForward);
    } else {
      arm.set(DoubleSolenoid.Value.kReverse);
    }
    out = !out;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
