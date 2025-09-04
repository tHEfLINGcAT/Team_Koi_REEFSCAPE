// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorSetPositionCommand extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final int angle;
  /**
   * Command to set (move to a preset relative angle) the elevator
   *
   * @param Elevator The subsystem used by this command.
   */
  public ElevatorSetPositionCommand(ElevatorSubsystem ele, int angle) {
    elevatorSubsystem = ele;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("setting position");
    elevatorSubsystem.SetElevatorPosition(angle);
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
