package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.drivetrain.DrivetrainMecanum;
import org.firstinspires.ftc.teamcode.utility.RobotConstants;

import java.util.Locale;

@TeleOp(name="Human Control", group="_main")
public class DriverControl extends LinearOpMode {

    // ---------------------------------
    // Class Variable(s)
    // ---------------------------------

    @Override
    public void runOpMode() throws InterruptedException {

        // -------------------------------------------------
        // Misc - OpMode Variables
        // -------------------------------------------------
        ElapsedTime opModeRunTime = new ElapsedTime();
        Pose2d initialPose = RobotConstants.OpModeTransition.getPoseFinalOpMode();

        // Road Runner -- need to implement in system
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ------------------------------------------------------------
        // Initialize System(s)
        // ------------------------------------------------------------
        DrivetrainMecanum drivetrain = new DrivetrainMecanum(hardwareMap, initialPose);

        // ------------------------------------------------------------
        // Variables for OpMode
        // ------------------------------------------------------------
        double inputAxial, inputLateral, inputYaw, restrictionExtensionMax;


        // Clear all telemetry
        telemetry.clearAll();

        // Repeat while opmode is in init mode (disable if using waitforstart)
        while (opModeInInit() && !isStopRequested()) {

            // ------------------------------------------------------------
            // Send telemetry message to signify robot completed initialization and waiting to start;
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------------");
            telemetry.addData("-", "All Systems Ready - Waiting to Start");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("run time", "%.1f seconds", opModeRunTime.seconds());
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("drivetrain", String.format(Locale.US,"{mode: %s, speed: %s}", drivetrain.getDrivetrainMode().getLabel(), drivetrain.getDrivetrainOutputPower().getLabel()));
            telemetry.addData("-","--------------------------------------");
//            telemetry.addData("light mode", sysLighting.getLightPatternCurrent().toString());

            // Show joystick information
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Controller Input");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("main", String.format(Locale.US,"{left X: %.3f, Left Y: %.3f, Right X: %.3f, Right Y: %.3f}", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y));
            telemetry.addData("alt", String.format(Locale.US,"{left X: %.3f, Left Y: %.3f, Right X: %.3f, Right Y: %.3f}", gamepad2.left_stick_x, gamepad2.left_stick_y, gamepad2.right_stick_x, gamepad2.right_stick_y));
            telemetry.addData("trigger", String.format(Locale.US,"{left: %.3f, Right: %.3f}", gamepad2.left_trigger, gamepad2.right_trigger));

            // Show imu / odometry information
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Inertia Measurement Unit");
            telemetry.addData("-","--------------------------------------");
//            telemetry.addData("imu", sysDrivetrain.getImuStatus());
//            telemetry.addData("heading", String.format(Locale.US,"{raw: %.3f, adj: %.3f}", sysDrivetrain.getRobotHeadingRaw(), sysDrivetrain.getRobotHeadingAdj()));
//            telemetry.addData("position", sysDrivetrain.getImuPositionDetail());
//            telemetry.addData("velocity", sysDrivetrain.getImuVelocityDetail());

            // ------------------------------------------------------------
            // - send telemetry to driver hub
            // ------------------------------------------------------------
            telemetry.update();
            idle();
        }

        // Wait for Start state (from driver station) - (disable if using an init loop)
//        waitForStart();


        // Reset runtime timer
        opModeRunTime.reset();

        // Clear all telemetry
        telemetry.clearAll();


        // Robot OpMode Loop
        while (opModeIsActive() && !isStopRequested()) {

            // ------------------------------------------------------------
            // Controls
            // ------------------------------------------------------------
            // Gamepad1 = Main Driver
            // ------------------------------------------------------------
            // -- Robot Movement
            // -- -- Axis (left_stick_x, left_stick_y): Drive
            // -- -- Axis (right_stick_x): Rotate
            //

            // ------------------------------------------------------------
            // Drivetrain
            // ------------------------------------------------------------
            // Assign gamepad control to motion in relation to:
            // -- gamepad input, direction
            // -- robot orientation to field
            // -- installed direction of control hub
            // -- orientation of drivetrain/motors
            inputYaw =  (gamepad1.right_stick_x);
            inputAxial = -(gamepad1.left_stick_y);
            inputLateral = (gamepad1.left_stick_x);

            // Update Odometry Reading(s)
            drivetrain.updateOdometry();

            // Endgame Notification
//            if(opModeRunTime.time() >= RobotConstants.CommonSettings.GameSettings.kEndgameStartTime && opModeRunTime.time() <= RobotConstants.CommonSettings.GameSettings.kEndgameEndTime) {
//                sysLighting.setLightPattern(RobotConstants.Lighting.Pattern.Default.kEndgame);
//            }
//            else if(opModeRunTime.time() >= RobotConstants.CommonSettings.GameSettings.kEndgameEndTime) {
//                sysLighting.setLightPattern(RobotConstants.Lighting.Pattern.Default.kEnd);
//            }

            // Drivetrain Type determined by 'Drivetrain Mode' enumeration selection (Default to Field Centric)
//            if(sysDrivetrain.getDrivetrainMode().equals(SysDrivetrain.DrivetrainMode.ROBOT_CENTRIC)) {
//
//                // Set Robot Centric Drivetrain
//                sysDrivetrain.driveMecanum(inputAxial, inputLateral, inputYaw, sysDrivetrain.getDrivetrainOutputPower().getValue());
//            }
//            else {

            // Set Field Centric Drivetrain
            drivetrain.driveMecanumFieldCentric(inputAxial, inputLateral, inputYaw, drivetrain.getDrivetrainOutputPower().getValue());
//            }

            // ------------------------------------
            // Drivetrain Speed Options
            // ------------------------------------
            if(gamepad1.right_bumper) {
                drivetrain.setDrivetrainOutputPower(DrivetrainMecanum.DrivetrainSpeed.MEDIUM);
            }

            if(!gamepad1.right_bumper) {
                drivetrain.setDrivetrainOutputPower(DrivetrainMecanum.DrivetrainSpeed.LOW);
            }

//            if(gamepad1.dpad_up) {
//                sysDrivetrain.setDrivetrainOutputPower(SysDrivetrainPinpoint.DrivetrainSpeed.HIGH);
//            }
//
//            if(gamepad1.dpad_down) {
//                sysDrivetrain.setDrivetrainOutputPower(SysDrivetrainPinpoint.DrivetrainSpeed.LOW);
//            }
//
//            if(gamepad1.dpad_left) {
//                sysDrivetrain.setDrivetrainOutputPower(SysDrivetrainPinpoint.DrivetrainSpeed.MEDIUM);
//            }
//
//            if(gamepad1.dpad_right) {
//                sysDrivetrain.setDrivetrainOutputPower(SysDrivetrainPinpoint.DrivetrainSpeed.SNAIL);
//            }

            // ------------------------------------
            // Drivetrain Mode Options
            // ------------------------------------
            if(gamepad1.start && gamepad1.dpad_up) {
                drivetrain.setDrivetrainMode(DrivetrainMecanum.DrivetrainMode.FIELD_CENTRIC);
            }

//            if(gamepad1.start && gamepad1.dpad_down) {
//                sysDrivetrain.setDrivetrainMode(SysDrivetrainPinpoint.DrivetrainMode.ROBOT_CENTRIC);
//            }



            // ------------------------------------
            // Override
            // ------------------------------------
            if(gamepad1.start && gamepad1.y) {

                // Reset the Robot Heading (normally done on init of Drivetrain system)
                drivetrain.resetZeroRobotHeading();
            }

            // ------------------------------------
            // Driver Hub Feedback
            // ------------------------------------
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Teleop - Main");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("run time", "%.1f seconds", opModeRunTime.seconds());
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("drivetrain", String.format(Locale.US,"{mode: %s, speed: %s}", drivetrain.getDrivetrainMode().getLabel(), drivetrain.getDrivetrainOutputPower().getLabel()));
            telemetry.addData("-","--------------------------------------");
//            telemetry.addData("light mode", sysLighting.getLightPatternCurrent().toString());
            telemetry.addData("-","--------------------------------------");

            // Show joystick information
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Controller Input");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("main", String.format(Locale.US,"{left X: %.3f, Left Y: %.3f, Right X: %.3f, Right Y: %.3f}", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y));
            telemetry.addData("alt", String.format(Locale.US,"{left X: %.3f, Left Y: %.3f, Right X: %.3f, Right Y: %.3f}", gamepad2.left_stick_x, gamepad2.left_stick_y, gamepad2.right_stick_x, gamepad2.right_stick_y));
            telemetry.addData("trigger", String.format(Locale.US,"{left: %.3f, Right: %.3f}", gamepad2.left_trigger, gamepad2.right_trigger));

            // Show imu / odometry information
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Inertia Measurement Unit");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("imu", drivetrain.getImuStatus());
            telemetry.addData("heading", String.format(Locale.US,"{raw: %.3f, adj: %.3f}", drivetrain.getRobotHeadingRaw(), drivetrain.getRobotHeadingAdj()));
            telemetry.addData("position", drivetrain.getImuPositionDetail());
            telemetry.addData("velocity", drivetrain.getImuVelocityDetail());

            // Show Arm and Intake Telemetry
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Arm / Intake");
            telemetry.addData("-","--------------------------------------");
//            telemetry.addData("arm position - motor", String.format(Locale.US,"{pivot: %d, extend left: %d, extend right: %d, shuttle: %d}", sysArm.getArmMotorCurrentPosition(RobotConstants.HardwareConfiguration.kLabelArmMotorPivot), sysArm.getArmMotorCurrentPosition(RobotConstants.HardwareConfiguration.kLabelArmMotorExtendLeft), sysArm.getArmMotorCurrentPosition(RobotConstants.HardwareConfiguration.kLabelArmMotorExtendRight), sysArm.getArmMotorCurrentPosition(RobotConstants.HardwareConfiguration.kLabelArmMotorShuttle)));
//            telemetry.addData("arm position - tolerance", String.format(Locale.US,"{pivot: %d, extend left: %d, extend right: %d}", sysArm.getArmMotorPositionTolerance(RobotConstants.HardwareConfiguration.kLabelArmMotorPivot), sysArm.getArmMotorPositionTolerance(RobotConstants.HardwareConfiguration.kLabelArmMotorExtendLeft), sysArm.getArmMotorPositionTolerance(RobotConstants.HardwareConfiguration.kLabelArmMotorExtendRight)));
//            telemetry.addData("arm position - extension", String.format(Locale.US,"{left: %.3f, right: %.3f, max: %.3f}", sysArm.getArmExtensionSensorCurrentPosition(RobotConstants.HardwareConfiguration.kLabelArmSensorExtendLimitLeft), sysArm.getArmExtensionSensorCurrentPosition(RobotConstants.HardwareConfiguration.kLabelArmSensorExtendLimitRight), sysArm.getArmExtensionPosition()));
//            telemetry.addData("intake position", String.format(Locale.US,"{rotate: %.3f, in left: %.3f, in right: %.3f}", sysIntake.getIntakeRotationPosition(), sysIntake.getIntakeCollectionPower(RobotConstants.HardwareConfiguration.kLabelIntakeServoCollectionLeft), sysIntake.getIntakeCollectionPower(RobotConstants.HardwareConfiguration.kLabelIntakeServoCollectionRight)));
//            telemetry.addData("arm extension - lower limit", sysArm.getArmLimitSensorTripped(RobotConstants.HardwareConfiguration.kLabelArmSensorExtendLimitLower));
//            telemetry.addData("arm pivot - lower limit", sysArm.getArmLimitSensorTripped(RobotConstants.HardwareConfiguration.kLabelArmSensorPivotLimitLower));
//            telemetry.addData("arm pivot - upper limit", sysArm.getArmLimitSensorTripped(RobotConstants.HardwareConfiguration.kLabelArmSensorPivotLimitUpper));
//            telemetry.addData("arm shuttle - lower limit", sysArm.getArmLimitSensorTripped(RobotConstants.HardwareConfiguration.kLabelArmSensorExtendLimitLower));
//            telemetry.addData("-","--------------------------------------");
//            telemetry.addData("-","-- Arm State");
//            telemetry.addData("-","--------------------------------------");
//            telemetry.addData("Extend Position", sysArm.getArmPositionMode(RobotConstants.HardwareConfiguration.kLabelArmMotorExtendLeft));
//            telemetry.addData("Extend Direction", sysArm.getArmTravelDirectionMode(RobotConstants.HardwareConfiguration.kLabelArmMotorExtendLeft));

            // Show Vision
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Vision");
            telemetry.addData("-","--------------------------------------");
//            telemetry.addData("alliance", String.format(Locale.US,"{color: %s, red: %d, blue: %d, green: %d}", sysVision.getAllianceColor(), sysVision.getAllianceColorValueRed(), sysVision.getAllianceColorValueBlue(), sysVision.getAllianceColorValueGreen()));

            // ------------------------------------------------------------
            // - send telemetry to driver hub
            // ------------------------------------------------------------
            telemetry.update();
        }




    }

}
