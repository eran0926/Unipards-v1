package frc.robot.SubSystem;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.RobotMap;

public class MusicSubSystem {
  private static Orchestra orchestra;
  private TalonFX mFrontLeftDriveMotor;
  private TalonFX mBackLeftDriveMotor;
  private TalonFX mBackRightDriveMotor;

  private TalonFX mFrontRightDriveMotor;

  private TalonFX mFrontLeftAngleMotor;
  private TalonFX mBackLeftAngleMotor;
  private TalonFX mBackRightAngleMotor;
  private TalonFX mFrontRightAngleMotor;

  public MusicSubSystem() {
    orchestra = new Orchestra();
    configAudio();
  }

  private void configAudio() {
    configAudioMotor();
    orchestra.loadMusic("flag_songchrp.chrp");
  }

  private void configAudioMotor() {
    mFrontLeftDriveMotor = new TalonFX(RobotMap.FRONT_LEFT_DRIVE_MOTOR_ID);
    mBackLeftDriveMotor = new TalonFX(RobotMap.BACK_LEFT_DRIVE_MOTOR_ID);
    mBackRightDriveMotor = new TalonFX(RobotMap.BACK_RIGHT_DRIVE_MOTOR_ID);
    mFrontRightDriveMotor = new TalonFX(RobotMap.FRONT_RIGHT_DRIVE_MOTOR_ID);
    mFrontLeftAngleMotor = new TalonFX(RobotMap.FRONT_LEFT_ANGLE_MOTOR_ID);
    mBackLeftAngleMotor = new TalonFX(RobotMap.BACK_LEFT_ANGLE_MOTOR_ID);
    mBackRightAngleMotor = new TalonFX(RobotMap.BACK_RIGHT_ANGLE_MOTOR_ID);
    mFrontRightAngleMotor = new TalonFX(RobotMap.FRONT_RIGHT_ANGLE_MOTOR_ID);
    orchestra.addInstrument(mFrontLeftDriveMotor);
    orchestra.addInstrument(mBackLeftDriveMotor);
    orchestra.addInstrument(mBackRightDriveMotor);
    orchestra.addInstrument(mFrontRightDriveMotor);
    orchestra.addInstrument(mFrontLeftAngleMotor);
    orchestra.addInstrument(mBackLeftAngleMotor);
    orchestra.addInstrument(mBackRightAngleMotor);
    orchestra.addInstrument(mFrontRightAngleMotor);
  }

  //  public void addSong(String song) {
//    orchestra.loadMusic(song);
//  }
  public void startMusic() {
    orchestra.play();
  }
  public void stopMusic() {
    orchestra.stop();
  }
  public void pauseMusic() {
    orchestra.pause();
  }
}
