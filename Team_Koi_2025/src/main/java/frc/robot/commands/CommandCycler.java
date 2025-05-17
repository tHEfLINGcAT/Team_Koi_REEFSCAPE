// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

/*a command that cycles through an array of commands, can be useful for preset cycling*/
public class CommandCycler extends Command {
  Command[] commands;
  int currentIndex = 0;
  /**
   * Creates a CommandCycler
   *
   * @param commands an array of the commands that shall be used
   */
  public CommandCycler(Command[] commands) {
    this.commands = commands;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    commands[currentIndex].execute();
    if(currentIndex >= commands.length - 1) {
      currentIndex = 0;
      return;
    }
    currentIndex++;
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
