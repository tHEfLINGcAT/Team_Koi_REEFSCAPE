// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RotationPresetSubsystem  extends SubsystemBase {
  private final SwerveSubsystem swerve;
  private final int[] headings;
  private int index = 0;

  public RotationPresetSubsystem(SwerveSubsystem swerve, int[] arr) {
      this.swerve = swerve;
      headings = arr;
  }

  public boolean rotateToCurrentHeading() {
      return swerve.rotateToHeading(headings[index]);
  }

  public boolean rotateToHeadingIndex(int i) {
    return swerve.rotateToHeading(headings[i]);
  }

  public void next() {
      index = (index + 1) % headings.length;
  }

  public void previous() {
      index = (index - 1 + headings.length) % headings.length;
  }

  public void reset() {
      index = 0;
  }
}
