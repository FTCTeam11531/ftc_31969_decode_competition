package org.firstinspires.ftc.teamcode.system.intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.system.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utility.RobotConstants;

import java.util.Arrays;
import java.util.List;

public class Intake {

//    public enum IntakeMode {
//
//
//    }

    // System OpMode
    private LinearOpMode opMode;

    // Intake Setting(s)


    // Define Hardware for subsystem
    private DcMotorEx intakeLeft, intakeRight;
    private List<DcMotorEx> listMotorIntake;

    // Constructor
    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {

        // Telemetry - Initialize - Start
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.addData(">", "System: Intake");
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.update();

        // Define and Initialize Motor(s)
//        intakeLeft = opMode.hardwareMap.get(DcMotorEx.class, RobotConstants.HardwareConfiguration.kLabelIntakeMotorLeft);
        intakeRight = opMode.hardwareMap.get(DcMotorEx.class, RobotConstants.HardwareConfiguration.kLabelIntakeMotorRight);

        listMotorIntake = Arrays.asList(intakeRight); //, intakeLeft);

        // Set common configuration for drive motor(s)
        for (DcMotorEx itemMotor : listMotorIntake) {

            // clone motor configuration
            MotorConfigurationType motorConfigurationType = itemMotor.getMotorType().clone();

            // Set motor configuration properties
            motorConfigurationType.setAchieveableMaxRPMFraction(RobotConstants.Intake.Configuration.kMotorAchievableMaxRpmFraction);

            // Write out motor configuration to motor
            itemMotor.setMotorType(motorConfigurationType);
        }

        // Set Zero Power mode for drivetrain
        setMotorZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Set Non-common motor configuration(s)
//        intakeLeft.setDirection(DcMotorEx.Direction.REVERSE);
        intakeRight.setDirection(DcMotorEx.Direction.REVERSE);

        // Telemetry - Initialize - End
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.addData(">", "System: Intake (Initialized)");
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.update();

    }

    // ----------------------------------------------
    // Action Methods
    // ----------------------------------------------

    public void activateIntake(double powerRight) {

        setMotorPower(powerRight);
    }

    public void deactivateIntake() {

        setMotorPower(0);
    }


    // ----------------------------------------------
    // Action Methods - Road Runner
    // ----------------------------------------------

    // Road Runner - Action - Activate Intake
    public class ActionActivateIntake implements Action {

        private double output;

        // Action class constructor
        public ActionActivateIntake(double output) {
            this.output = output;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            activateIntake(output);
            return false;
        }
    }

    public Action actionActivateIntake(double output) {
        return new ActionActivateIntake(output);
    }


    // ----------------------------------------------
    // Get Methods
    // ----------------------------------------------

    public double getMotorPower(String hardwareLabel) {
        double outputPower;

        switch (hardwareLabel) {
            case RobotConstants.HardwareConfiguration.kLabelIntakeMotorLeft:
                outputPower = intakeLeft.getPower();
                break;

            case RobotConstants.HardwareConfiguration.kLabelIntakeMotorRight:
                outputPower = intakeRight.getPower();
                break;

            default:
                outputPower = 0;
        }

        return outputPower;
    }

    public double getMotorVelocity(String hardwareLabel) {
        double outputVelocity;

        switch (hardwareLabel) {
            case RobotConstants.HardwareConfiguration.kLabelIntakeMotorLeft:
                outputVelocity = intakeLeft.getVelocity();
                break;

            case RobotConstants.HardwareConfiguration.kLabelIntakeMotorRight:
                outputVelocity = intakeRight.getVelocity();
                break;

            default:
                outputVelocity = 0;
        }

        return outputVelocity;
    }

//    public Shooter.ShootingMode getShootingModeLeft() {
//        return shootingModeLeft;
//    }
//
//    public Shooter.ShootingMode getShootingModeRight() {
//        return shootingModeRight;
//    }


    // ----------------------------------------------
    // Set Methods
    // ----------------------------------------------

    public void setMotorRunMode(DcMotorEx.RunMode inRunMode) {
        for (DcMotorEx itemMotor: listMotorIntake) {
            itemMotor.setMode(inRunMode);
        }
    }
    public void setMotorZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior inZeroPowerBehavior) {
        for (DcMotorEx itemMotor : listMotorIntake) {
            itemMotor.setZeroPowerBehavior(inZeroPowerBehavior);
        }
    }

    private void setMotorPower(double powerRight) { //, double powerLeft) {

        // Send calculated power to wheels
//        intakeLeft.setPower(powerLeft);
        intakeRight.setPower(powerRight);
    }

//    public void setShootingModeLeft(Shooter.ShootingMode newShootingMode) {
//        shootingModeLeft = newShootingMode;
//    }



}
