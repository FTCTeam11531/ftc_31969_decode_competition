package org.firstinspires.ftc.teamcode.system.kickstand;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.RobotConstants;

import java.util.List;

public class Kickstand {

    // System OpMode
    private LinearOpMode opMode;

    // Kickstand Setting(s)


    // Define Hardware for subsystem
    private Servo kickstand;

    // Constructor
    public Kickstand(LinearOpMode opMode) { this.opMode = opMode; }


    public void init() {

        // Telemetry - Initialize - Start
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.addData(">", "System: Indexer");
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.update();

        // Define and Initialize Motor(s)

        // Define and Initialize Servo(s)
        kickstand = opMode.hardwareMap.get(Servo.class, RobotConstants.HardwareConfiguration.kLabelServoKickstand);

        // Initialize Kickstand
        kickstand.setPosition(RobotConstants.Kickstand.Setpoint.kInitial);
    }


    // ----------------------------------------------
    // Action Method(s)
    // ----------------------------------------------


    // ----------------------------------------------
    // Get Method(s)
    // ----------------------------------------------
    public double getKickstandPosition() {
        return kickstand.getPosition();
    }

    // ----------------------------------------------
    // Set Method(s)
    // ----------------------------------------------
    public void setKickstandPosition(double setpoint) {
        kickstand.setPosition(setpoint);
    }




}
