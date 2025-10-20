package org.firstinspires.ftc.teamcode.system.indexer;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utility.RobotConstants;

public class Indexer {

//    public enum IndexerMode {
//
//    }

    // System OpMode
    private LinearOpMode opMode;

    // Intake Setting(s)


    // Define Hardware for subsystem
    private CRServo indexLeft, indexRight;

    // Constructor
    public Indexer(LinearOpMode opMode) { this.opMode = opMode; }

    public void init() {

        // Telemetry - Initialize - Start
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.addData(">", "System: Indexer");
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.update();

        // Define and Initialize Motor(s)

        // Define and Initialize Servo(s)
        indexLeft = opMode.hardwareMap.get(CRServo.class, RobotConstants.HardwareConfiguration.kLabelIndexServoLeft);
        indexRight = opMode.hardwareMap.get(CRServo.class, RobotConstants.HardwareConfiguration.kLabelIndexServoRight);

        indexLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        indexRight.setDirection(DcMotorSimple.Direction.FORWARD);

        indexLeft.setPower(0);
        indexRight.setPower(0);

        // Define and Initialize Sensor(s)


    }

    // ----------------------------------------------
    // Action Method(s)
    // ----------------------------------------------
    public void activateIndexer(String hardwareLabel, double setpoint) {

        switch (hardwareLabel) {
            case RobotConstants.HardwareConfiguration.kLabelIndexServoLeft:
                indexLeft.setPower(setpoint);
                break;

            case RobotConstants.HardwareConfiguration.kLabelIndexServoRight:
                indexRight.setPower(setpoint);
                break;

        }

    }

    public void deactivateIndexer(String hardwareLabel) {
        double setpoint = 0;

        activateIndexer(hardwareLabel, setpoint);
    }

    // ----------------------------------------------
    // Action Methods - Road Runner
    // ----------------------------------------------

    // Road Runner - Action - Activate Indexer
    public class ActionActivateIndexer implements Action {

        private String hardwareLabel;
        private double setpoint;

        // Action class constructor
        public ActionActivateIndexer(String hardwareLabel, double setpoint) {
            this.hardwareLabel = hardwareLabel;
            this.setpoint = setpoint;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            activateIndexer(hardwareLabel, setpoint);
            return false;
        }
    }

    public Action actionActivateIndexer(String hardwareLabel, double setpoint) {
        return new ActionActivateIndexer(hardwareLabel, setpoint);
    }


    // ----------------------------------------------
    // Get Method(s)
    // ----------------------------------------------
    public double getIndexerPower(String hardwareLabel) {
        double IndexerPower;

        switch (hardwareLabel) {
            case RobotConstants.HardwareConfiguration.kLabelIndexServoLeft:
                IndexerPower = indexLeft.getPower();
                break;

            case RobotConstants.HardwareConfiguration.kLabelIndexServoRight:
                IndexerPower = indexRight.getPower();
                break;

            default:
                IndexerPower = 0;
        }

        return IndexerPower;
    }

    // ----------------------------------------------
    // Set Method(s)
    // ----------------------------------------------


}
