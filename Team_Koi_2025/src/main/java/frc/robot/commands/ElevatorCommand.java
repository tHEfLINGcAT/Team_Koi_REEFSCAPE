// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class ElevatorCommand extends Command {
  private final ElevatorSubsystem m_subsystem;
  private final DoubleSupplier joystickInput;

  public ElevatorCommand(ElevatorSubsystem subsystem, DoubleSupplier joystickInput) {
    this.m_subsystem = subsystem;
    this.joystickInput = joystickInput;
    addRequirements(subsystem);
  }

  
  //public void initialize() {
   // m_subsystem.setTargetPosition(Constants.ElevatorConstants.ELEVATOR_START_POSITION);
  //}

  // CHAT DID I DO IT? DID I COOK?

  @Override
  public void execute() {
      double speed = (joystickInput.getAsDouble());
      m_subsystem.setTargetVelocity(speed);
    //  System.out.println("IM RUNNINGGG:"+(speed));
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
    if (interrupted) {
      System.out.println("INTURAPTED");
    }
   }
}