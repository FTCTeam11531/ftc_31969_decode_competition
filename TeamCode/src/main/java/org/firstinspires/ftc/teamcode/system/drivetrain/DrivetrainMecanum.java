package org.firstinspires.ftc.teamcode.system.drivetrain;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.RobotConstants;

import java.util.Locale;

public class DrivetrainMecanum extends MecanumDrive {


    // Enum: Drive mode
    public enum DrivetrainMode {

        FIELD_CENTRIC {

            // Label for setting
            @Override
            public String getLabel() { return "Field Centric"; }

        },
        ROBOT_CENTRIC {

            // Label for setting
            @Override
            public String getLabel() { return "Robot Centric"; }

        };

        public abstract String getLabel();
    }

    // Enum: Drive Speed
    public enum DrivetrainSpeed {

        HIGH {

            // Value for setting
            @Override
            public double getValue() { return RobotConstants.Drivetrain.Configuration.kMotorOutputPowerHigh; }

            // Label for setting
            @Override
            public String getLabel() { return "High"; }

        },
        MEDIUM {

            // Value for setting
            @Override
            public double getValue() { return RobotConstants.Drivetrain.Configuration.kMotorOutputPowerMedium; }

            // Label for setting
            @Override
            public String getLabel() { return "Medium"; }

        },
        LOW {

            // Value for setting
            @Override
            public double getValue() { return RobotConstants.Drivetrain.Configuration.kMotorOutputPowerLow; }

            // Label for setting
            @Override
            public String getLabel() { return "Low"; }

        },
        SNAIL {

            // Value for setting
            @Override
            public double getValue() { return RobotConstants.Drivetrain.Configuration.kMotorOutputPowerSnail; }

            // Label for setting
            @Override
            public String getLabel() { return "Snail"; }

        };

        public abstract double getValue();
        public abstract String getLabel();
    }

    // System Opmode - Calling opmode
    private LinearOpMode sysOpMode;

    // Drivetrain Settings
    private DrivetrainMode drivetrainMode = DrivetrainMode.FIELD_CENTRIC;
    private DrivetrainSpeed drivetrainSpeed = DrivetrainSpeed.MEDIUM;



    public static class Params {

    }

    public static Params drivetrainParameters = new Params();



    public DrivetrainMecanum(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
    }


    // -----------------------------------------
    // Action Method(s)
    // -----------------------------------------

    /**
     * <h2>Drivetrain Method: driveMecanumFieldCentric</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#kAboutAuthorName}<br>
     * <b>Season Game:</b> {@value RobotConstants.About#kAboutSeasonGameName}<br>
     * <hr>
     * <p><br>
     * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
     * Each motion axis is controlled by one Joystick axis.
     * </p>
     * <br>
     * <p>
     * This is a 'Field Centric' variation of the mecanum drivetrain
     * </p>
     * <br>
     * <i>[Y]</i> <b>Axial:</b>   Driving forward and backward<br>
     * <i>[X]</i> <b>Lateral:</b> Strafing right and left<br>
     * <i>[R]</i> <b>Yaw:</b>   Rotating Clockwise and counter clockwise<br>
     *
     * @param inputAxial   [Y] Driving forward and backward
     * @param inputLateral [X] Strafing right and left
     * @param inputYaw     [R] Rotating Clockwise and counter clockwise
     * @param maxOutputPowerPercent Percent of power to apply to motors
     *
     * <br>
     */
    public void driveMecanumFieldCentric(double inputAxial, double inputLateral, double inputYaw, double maxOutputPowerPercent) {

        double modMaintainMotorRatio;

        // Axial = input y
        // Lateral = input x
        // Yaw = rotational

        double modAxial = (inputAxial * maxOutputPowerPercent);  // Note: pushing stick forward gives negative value
        double modLateral = (inputLateral * maxOutputPowerPercent);
        double modYaw = (inputYaw * maxOutputPowerPercent);

        // Get heading value from the IMU
//        updateOdometry();
        double botHeading = getRobotHeadingOnBoard();

        // Adjust the lateral and axial movements based on heading
        double adjLateral = modLateral * Math.cos(-botHeading) - modAxial * Math.sin(-botHeading);
        double adjAxial = modLateral * Math.sin(-botHeading) + modAxial * Math.cos(-botHeading);

        adjLateral = adjLateral * RobotConstants.Drivetrain.Configuration.kMotorLateralMovementStrafingCorrection; // Mod to even out strafing

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        modMaintainMotorRatio = Math.max(Math.abs(modAxial) + Math.abs(modLateral) + Math.abs(modYaw), RobotConstants.Drivetrain.Configuration.kMotorOutputPowerMax);

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = (adjAxial + adjLateral + modYaw) / modMaintainMotorRatio;
        double rightFrontPower = (adjAxial - adjLateral - modYaw) / modMaintainMotorRatio;
        double leftBackPower   = (adjAxial - adjLateral + modYaw) / modMaintainMotorRatio;
        double rightBackPower  = (adjAxial + adjLateral - modYaw) / modMaintainMotorRatio;

        // Use existing function to drive both wheels.
        setDriveMotorPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

//    public void driveMecanumVectorWeighted(Pose2d drivePose) {
//        Pose2d newPose = drivePose;
//
//        if(Math.abs(drivePose.position.x)
//                + Math.abs(drivePose.position.y)
//                + Math.abs(drivePose.heading.toDouble()) > 1) {
//
//            double adjDivisor = 1 * Math.abs(drivePose.position.x)
//                    + 1 * Math.abs(drivePose.position.y)
//                    + 1 * Math.abs(drivePose.heading.toDouble());
//
//            newPose = new Pose2d(
//                      1 * drivePose.position.x
//                    , 1 * drivePose.position.y
//                    , 1 * drivePose.heading.toDouble()
//            ).
//
//            )
//
//        }
//
//    }

    /**
     * <h2>Drivetrain Method: driveMecanum</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#kAboutAuthorName}<br>
     * <b>Season:</b> {@value RobotConstants.About#kAboutSeasonGameName}<br>
     * <hr>
     * <p>
     * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
     * Each motion axis is controlled by one Joystick axis.
     * </p>
     * <p>
     * This is a standard mecanum drivetrain or a 'Robot Centric' drivetrain
     * </p>
     * <br>
     * <i>[Y]</i> <b>Axial:</b>   Driving forward and backward<br>
     * <i>[X]</i> <b>Lateral:</b> Strafing right and left<br>
     * <i>[R]</i> <b>Yaw:</b>   Rotating Clockwise and counter clockwise<br>
     *
     * @param inputAxial   [Y] Driving forward and backward
     * @param inputLateral [X] Strafing right and left
     * @param inputYaw     [R] Rotating Clockwise and counter clockwise
     * @param maxOutputPowerPercent Percent of power to apply to motors
     *
     * <br>
     */
    public void driveMecanum(double inputAxial, double inputLateral, double inputYaw, double maxOutputPowerPercent) {

        double modMaintainMotorRatio;

        double modAxial   = (inputAxial * maxOutputPowerPercent);  // Note: pushing stick forward gives negative value
        double modLateral = (inputLateral * maxOutputPowerPercent) * RobotConstants.Drivetrain.Configuration.kMotorLateralMovementStrafingCorrection; // Mod to even out strafing
        double modYaw     = (inputYaw * maxOutputPowerPercent);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        modMaintainMotorRatio = Math.max(Math.abs(modAxial) + Math.abs(modLateral) + Math.abs(modYaw), RobotConstants.Drivetrain.Configuration.kMotorOutputPowerMax);

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = (modAxial + modLateral + modYaw) / modMaintainMotorRatio;
        double rightFrontPower = (modAxial - modLateral - modYaw) / modMaintainMotorRatio;
        double leftBackPower   = (modAxial - modLateral + modYaw) / modMaintainMotorRatio;
        double rightBackPower  = (modAxial + modLateral - modYaw) / modMaintainMotorRatio;

        // Use existing function to drive both wheels.
        setDriveMotorPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }


    public void updateOdometry() {
//        pinpoint.update();
        localizer.update();
    }

    /**
     *
     *
     */
    public void resetZeroRobotHeading() {

        // Set the Heading Offset to the IMU raw heading
//        pinpoint.resetPosAndIMU();
        lazyImu.get().resetYaw();
        localizer.resetRobotHeading();
    }

    // -----------------------------------------
    // Get Method(s)
    // -----------------------------------------
    public String getImuStatus() {
//        return pinpoint.getDeviceStatus().toString();
        return lazyImu.get().getConnectionInfo();
    }

    public Pose2d getRobotPosition() {
//        return pinpoint.getPosition();
        return localizer.getPose();
    }

    public Pose2d getRobotPositionRR() {
//        return pinpoint.getPositionRR();
        return localizer.getPose();
    }

    public Pose2d getRobotVelocity() {
//        return pinpoint.getVelocity();
        return localizer.getPose();
    }

    public String getImuPositionDetail() {
        return String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", getRobotPosition().position.x, getRobotPosition().position.y, getRobotPosition().heading.real);
    }

    public String getImuVelocityDetail() {
        return String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", getRobotVelocity().position.x, getRobotVelocity().position.y, getRobotVelocity().heading.real);
    }

    /**
     * <h2>Drivetrain Method: getRobotHeadingRaw</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#kAboutAuthorName}<br>
     * <b>Season Game:</b> {@value RobotConstants.About#kAboutSeasonGameName}<br>
     * <hr>
     * <p>
     * Get the output heading value from the IMU.
     * </p>
     * @return double - Output heading value from the IMU - raw reading
     */
    public double getRobotHeadingRaw() {
//        return pinpoint.getHeading();
        return localizer.getPose().heading.real;
    }

    public double getRobotHeadingOnBoard() {
        return lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**
     * <h2>Drivetrain Method: getRobotHeadingAdj</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#kAboutAuthorName}<br>
     * <b>Season Game:</b> {@value RobotConstants.About#kAboutSeasonGameName}<br>
     * <hr>
     * <p>
     * Get the output heading value from the IMU (with offset adjustment).
     * </p>
     * @return double - Output heading value from the IMU (with offset adjustment)
     * <br>
     */
    public double getRobotHeadingAdj() {

        // Variable for output heading value
        double outRobotHeadingValue;


        // TODO: Check on input directional inversion... may not need the inversion here!

        // Get heading value from the IMU
        // Read inverse IMU heading, as the IMU heading is CW positive
        outRobotHeadingValue = getRobotHeadingRaw() + RobotConstants.Drivetrain.Odometry.Transition.getImuTransitionAdjustment();

        // Should the IMU heading be inversed? Does it matter?
        // Will need to view the heading readout on the driver hub

        //imuUnit.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle

        return outRobotHeadingValue;
    }

    public DrivetrainMode getDrivetrainMode() {
        return drivetrainMode;
    }

    public DrivetrainSpeed getDrivetrainOutputPower() {
        return drivetrainSpeed;
    }

    // -----------------------------------------
    // Set Method(s)
    // -----------------------------------------

    public void setPose(Pose2d pose) {
        localizer.setPose(pose);
    }


    /**
     * <h2>Drivetrain Method: setDriveMotorPower</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#kAboutAuthorName}<br>
     * <b>Season Game:</b> {@value RobotConstants.About#kAboutSeasonGameName}<br>
     * <hr>
     * <p>
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     * </p>
     * @param powerLeftFront  Power to Left Front Wheel
     * @param powerRightFront Power to Right Front Wheel
     * @param powerLeftBack   Power to Left Back Wheel
     * @param powerRightBack  Power to Right Back Wheel
     *
     * <br>
     */
    private void setDriveMotorPower(double powerLeftFront, double powerRightFront, double powerLeftBack, double powerRightBack) {

        // Send calculated power to wheels
        leftFront.setPower(powerLeftFront);
        rightFront.setPower(powerRightFront);
        leftBack.setPower(powerLeftBack);
        rightBack.setPower(powerRightBack);
    }

//
//    /**
//     * <h2>Drivetrain Method: setDriveMotorZeroPowerBehavior</h2>
//     * <hr>
//     * <b>Author: </b><br>
//     * <b>Season: </b><br>
//     * <hr>
//     * <p><br>
//     * Set the behavior of each drivetrain motor when zero power is supplied to the motor(s).<br>
//     * - Mode(s): Brake / Coast<br>
//     * </p>
//     *
//     * @param zeroPowerMode DcMotorEx.ZeroPowerBehavior - Behavior when zero power sent to motor
//     *
//     */
//    public void setDriveMotorZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerMode) {
//
//        for (DcMotorEx itemMotor : listMotorDrivetrain) {
//            itemMotor.setZeroPowerBehavior(zeroPowerMode);
//        }
//    }


    public void setDrivetrainMode(DrivetrainMode newDrivetrainMode) {
        drivetrainMode = newDrivetrainMode;
    }

    public void setDrivetrainOutputPower(DrivetrainSpeed newDrivetrainSpeed) {
        drivetrainSpeed = newDrivetrainSpeed;
    }


}
