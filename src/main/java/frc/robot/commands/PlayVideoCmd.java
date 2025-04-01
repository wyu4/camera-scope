// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SimulationVideoConstants;
import frc.robot.subsystems.VideoPlayer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PlayVideoCmd extends Command {
  private final VideoPlayer videoPlayer;
  private final Timer timer = new Timer();
  private final List<Pose3d> poses;
  private final boolean logAscii;
  private int currentFrame = 0;

  private Double lastFrameTimestamp;

  /** Creates a new PlayVideo. */
  public PlayVideoCmd(VideoPlayer videoPlayer) {
    this(videoPlayer, videoPlayer.getPoses());
  }

  /** Creates a new PlayVideo. */
  public PlayVideoCmd(VideoPlayer videoPlayer, List<Pose3d> poses) {
    this(videoPlayer, poses, SimulationVideoConstants.kLogAscii);
  }

  /** Creates a new PlayVideo. */
  public PlayVideoCmd(VideoPlayer videoPlayer, List<Pose3d> poses, boolean logAscii) {
    this.videoPlayer = videoPlayer;
    this.poses = poses;
    this.logAscii = logAscii;
    Logger.recordOutput("VideoPlayer/Display/Head", currentFrame);
    addRequirements(videoPlayer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    videoPlayer.resetPoses();
    currentFrame = 0;
    lastFrameTimestamp = null;
    timer.restart();
    Logger.recordOutput("VideoPlayer/Display/Head", currentFrame);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!videoPlayer.isAllowed()) {
      return;
    }

    if (lastFrameTimestamp == null) {
      lastFrameTimestamp = Timer.getFPGATimestamp();
      return;
    }

    if (timer.hasElapsed(SimulationVideoConstants.kDisplayDeltaSeconds)) {
      timer.restart();
      Boolean[][] status = videoPlayer.getChunkStatus(currentFrame);
      Double[][] degrees = videoPlayer.getChunkDegress(currentFrame);
      String consoleScreen = "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
      for (int y = 0; y < status.length; y++) {
        consoleScreen += "\n";
        Boolean[] row = status[y];
        for (int x = 0; x < row.length; x++) {
          Pose3d defaultPose = videoPlayer.calculateDefaultPose(x, y);
          Boolean chunkEnabled = (row[x] == null ? false : row[x]);

          poses.set((y * SimulationVideoConstants.kDisplayColumns) + x,
              chunkEnabled ? new Pose3d(defaultPose.getX(), defaultPose.getY(), 0, new Rotation3d(0, 0, degrees[y][x]))
                  : new Pose3d(defaultPose.getX() + SimulationVideoConstants.kDisplayWidthMeters + 100,
                      defaultPose.getY() + SimulationVideoConstants.kDisplayHeightMeters + 100, 100,
                      defaultPose.getRotation()));
          consoleScreen += chunkEnabled ? "#" : " ";
        }
      }
      logAscii(currentFrame + "\n" + consoleScreen);
      currentFrame++;

      
      Logger.recordOutput("VideoPlayer/Display/Head", currentFrame);
      Logger.recordOutput("VideoPlayer/Display/RealDeltaSeconds", Timer.getFPGATimestamp() - lastFrameTimestamp);
      lastFrameTimestamp = Timer.getFPGATimestamp();
    }
  }

  private void logAscii(String output) {
    if (logAscii) {
      System.out.println(output);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentFrame >= videoPlayer.getFrameCount() || !videoPlayer.isAllowed();
  }
}
