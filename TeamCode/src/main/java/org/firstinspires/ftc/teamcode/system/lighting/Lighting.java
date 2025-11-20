package org.firstinspires.ftc.teamcode.system.lighting;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.RobotConstants;

public class Lighting {


    public enum LightMode {

        DEFAULT,
        DEFAULT_TELEOP,
        DEFAULT_AUTONOMOUS,
        ON_TARGET_BLUE,
        ON_TARGET_RED,
        TELEOP_ENDGAME,
        TELEOP_MATCH_END,
        AUTO_BLUE_ALLIANCE,
        AUTO_RED_ALLIANCE,
        AUTO_PARK
    }

    // System OpMode
    private LinearOpMode opMode;

    // Intake Setting(s)


    // Define Hardware for subsystem
    //  Controller(s)
    private RevBlinkinLedDriver lightController;
    private RevBlinkinLedDriver.BlinkinPattern lightPattern;

    // Allow Enable or Disable for lighting
    private boolean enableLighting = RobotConstants.Lighting.kEnable;

    // Constructor
    public Lighting(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {

        // Telemetry - Initialize - Start
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.addData(">", "System: Intake");
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.update();

        if(enableLighting) {
            lightController = opMode.hardwareMap.get(RevBlinkinLedDriver.class, RobotConstants.HardwareConfiguration.kLabelLightController);

            // Set Initial Light pattern
            lightPattern = RobotConstants.Lighting.Pattern.kDefault;
            setLightPattern(getLightPatternCurrent());
        }

        // Telemetry - Initialize - End
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.addData(">", "System: Intake (Initialized)");
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.update();

    }


    // -----------------------------------------
    // Action Method(s)
    // -----------------------------------------


    // -----------------------------------------
    // Action Method(s) - Road Runner
    // -----------------------------------------

    // Road Runner - Action - Set Lighting Effect
    public class ActionSetLightEffect implements Action {

        private RevBlinkinLedDriver.BlinkinPattern lightPattern;

        // Action Class Constructor
        public ActionSetLightEffect(RevBlinkinLedDriver.BlinkinPattern lightPattern) {
            this.lightPattern = lightPattern;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setLightPattern(lightPattern);
            return false;
        }
    }

    public Action actionSetLightEffect(RevBlinkinLedDriver.BlinkinPattern lightPattern) {
        return new ActionSetLightEffect(lightPattern);
    }


    // -----------------------------------------
    // Get Method(s)
    // -----------------------------------------
    public RevBlinkinLedDriver.BlinkinPattern getLightPatternCurrent() {

        return lightPattern;
    }

    public RevBlinkinLedDriver.BlinkinPattern getLightPatternNext() {

        return lightPattern.next();
    }

    public RevBlinkinLedDriver.BlinkinPattern getLightPatternPrevious() {

        return lightPattern.previous();
    }


    // -----------------------------------------
    // Set Method(s)
    // -----------------------------------------
    public void setLightPattern(RevBlinkinLedDriver.BlinkinPattern lightPattern) {

        // Only change pattern when lighting system is enabled
        if(enableLighting) {
            lightController.setPattern(lightPattern);
        }

    }

}
