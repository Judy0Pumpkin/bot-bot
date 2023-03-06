// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.DelayQueue;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Arm extends SubsystemBase {

 DoubleSolenoid mArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
 DoubleSolenoid mClaw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
 DoubleSolenoid mTube = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
 Compressor mCompressor= new Compressor(0, PneumaticsModuleType.CTREPCM);
 Timer mTimer = new Timer();
 
 boolean x = true;
 boolean y = true;
 boolean z = true;

    public Arm (){
      mCompressor.enableDigital();

      mClaw.set(DoubleSolenoid.Value.kForward);
      mTube.set(DoubleSolenoid.Value.kForward);
      mArm.set(DoubleSolenoid.Value.kReverse);
    }

    public void periodic(){

      
    }
    public void armOut(){
     mArm.set(DoubleSolenoid.Value.kForward);
    }    
    public void armIn(){
      mArm.set(DoubleSolenoid.Value.kReverse);
    }  
    public void clawrelease(){
      mClaw.set(DoubleSolenoid.Value.kReverse);
    }

    public void clawForward(){
      mClaw.set(DoubleSolenoid.Value.kForward);
    }
    public void tubeForward(){
      mTube.set(DoubleSolenoid.Value.kForward);
    }  

    public void tubeIn(){
      mTube.set(DoubleSolenoid.Value.kReverse);
    }  
   
    }  

