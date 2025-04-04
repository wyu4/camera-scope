// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class FieldConstants {
    public static final AprilTagFieldLayout kfieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final double kFieldLengthX = kfieldLayout.getFieldLength();
    public static final double kFieldWidthY = kfieldLayout.getFieldWidth();
  }

  public static class SimulationVideoConstants {
    public static final String kInputSourceLocation = "BadApple.mp4"; // Video path relative to the deploy folder.
    public static final int kDisplayColumns = 48; // Number of chunks on x-axis
    public static final int kDisplayRows = 36; // Numbers of chunks on y-axis
    public static final double kDisplayOffsetX = 0; // Offset X
    public static final double kDisplayOffsetY = 0; // Offset Y
    public static final double kDisplayGapMeters = 0.5; // Gap between chunks
    public static final double kDisplayDeltaSeconds = 1.0/30.0; // FPS

    public static final boolean kRotations = true; // Enables rotations. Will increase processing time.
    public static final boolean kInverted = false; // Invert enabled/disabled chunks
    public static final boolean kLogAscii = false; // Log an ascii representation of which chunks are enabled

    // REAL display dimensions. Helps with alignments
    public static final double kDisplayWidthMeters = (kDisplayColumns - 1) * kDisplayGapMeters;
    public static final double kDisplayHeightMeters = (kDisplayRows - 1) * kDisplayGapMeters;
  }
}
