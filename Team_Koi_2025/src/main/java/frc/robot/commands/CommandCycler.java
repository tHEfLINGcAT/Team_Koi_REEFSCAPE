package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class CommandCycler {
  private final Command[] commands;
  private int currentIndex = 0;
  private Command currentRunning = null;

  public CommandCycler(Command... commands) {
    this.commands = commands;
  }

  public void cycle() {
    // Cancel current command if it's still running
    if (currentRunning != null && currentRunning.isScheduled()) {
      currentRunning.cancel();
    }

    // Schedule the next command
    currentRunning = commands[currentIndex];
    CommandScheduler.getInstance().schedule(currentRunning);

    // Increment and wrap around
    currentIndex = (currentIndex + 1) % commands.length;
  }
}