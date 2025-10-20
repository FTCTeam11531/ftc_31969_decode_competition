package org.firstinspires.ftc.teamcode.system.sound;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.RobotConstants;

public class Sound {

    // System OpMode
    private LinearOpMode opMode;

    SoundPlayer soundPlayer;
    SoundPlayer.PlaySoundParams soundPlayerParms;

    // Constructor
    public Sound(LinearOpMode opMode) { this.opMode = opMode; }

    public void init() {

        // Telemetry - Initialize - Start
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.addData(">", "System: Sound");
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.update();

        // Setup the SoundPlayer
        soundPlayer = SoundPlayer.getInstance();

        // Sounds player parameters
        soundPlayerParms = new SoundPlayer.PlaySoundParams();
        soundPlayerParms.loopControl = 0;
        soundPlayerParms.waitForNonLoopingSoundsToFinish = true;
        soundPlayerParms.rate = 1.0f;
        soundPlayerParms.volume = RobotConstants.Sound.kSoundVolumeDefault;

        // Telemetry - Initialize - End
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.addData(">", "System: Sound (Initialized)");
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.update();

    }


    // ----------------------------------------------
    // Action Methods
    // ----------------------------------------------
    public void playSoundFileByName(String soundFileName) {

        if(checkSoundFileStatus(getSoundFileIdByName(soundFileName))) {
            soundPlayer.setMasterVolume(RobotConstants.Sound.kSoundVolumnSetpoint);

            soundPlayer.startPlaying(
                      opMode.hardwareMap.appContext
                    , getSoundFileIdByName(soundFileName)
                    , soundPlayerParms
                    , null
                    , new Runnable() {
                        @Override
                        public void run() {
                            soundPlayer.setMasterVolume(RobotConstants.Sound.kSoundVolumnSetpoint);
                        }
                    }
            );
        }
    }

    public void stopAllSoundPlayback() {

        // Stop playing all active sounds
        soundPlayer.stopPlayingAll();
        soundPlayer.stopPlayingLoops();
    }

    public boolean checkSoundFileStatus(int soundFileId) {
        boolean isSoundFileFound = false;

        if(soundFileId != 0) {
            isSoundFileFound = soundPlayer.preload(opMode.hardwareMap.appContext, soundFileId);
        }

        return isSoundFileFound;
    }

    // ----------------------------------------------
    // Get Methods
    // ----------------------------------------------
    public int getSoundFileIdByName(String soundFileName) {

        return opMode.hardwareMap.appContext.getResources().getIdentifier(
                  soundFileName
                , "raw"
                , opMode.hardwareMap.appContext.getPackageName()
        );
    }

    // ----------------------------------------------
    // Set Methods
    // ----------------------------------------------


}
