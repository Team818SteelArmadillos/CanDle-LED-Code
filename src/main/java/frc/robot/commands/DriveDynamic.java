// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CanDle;
public class DriveDynamic extends Command {

  CanDle m_MotorSubsystem;
  public DriveDynamic(CanDle MotorSubsystem) {
    m_MotorSubsystem = MotorSubsystem;
    addRequirements(MotorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_MotorSubsystem.setLed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  // if ((Math.abs(rightSpeed.get())>0.05) & (Math.abs(rightSpeed.get())>0.05)){
    // m_MotorSubsystem.setMotorSpeedDyn(rightSpeed.get(), (leftSpeed.get()));
  // } else {
    
  // }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
