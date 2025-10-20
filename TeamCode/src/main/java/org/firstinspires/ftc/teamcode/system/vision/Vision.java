package org.firstinspires.ftc.teamcode.system.vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.utility.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

import kotlin.Triple;

public class Vision {

    // System OpMode
    private LinearOpMode opMode;

    // Camera(s)
    private HuskyLens cameraAIFront;
    private WebcamName cameraAprilTag;

    private OpenCvWebcam cameraStream;

    // Sensor(s)
    private ColorSensor sensorAllianceTag;

    // Camera Setting(s)
    private Position positionCameraAprilTag = new Position(
              DistanceUnit.INCH
            , RobotConstants.Vision.CameraAprilTag.Pose.kPositionX
            , RobotConstants.Vision.CameraAprilTag.Pose.kPositionY
            , RobotConstants.Vision.CameraAprilTag.Pose.kPositionZ
            , 0);

    private YawPitchRollAngles orientationCameraAprilTag = new YawPitchRollAngles(
            AngleUnit.DEGREES
            , RobotConstants.Vision.CameraAprilTag.Pose.kOrientationYaw
            , RobotConstants.Vision.CameraAprilTag.Pose.kOrientationPitch
            , RobotConstants.Vision.CameraAprilTag.Pose.kOrientationRoll
            , 0);

    // April Tag Processing
    private AprilTagProcessor processorAprilTag;
    private VisionPortal visionPortalAprilTag;

    public Vision(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {

        // Camera
        cameraAIFront = opMode.hardwareMap.get(HuskyLens.class, RobotConstants.HardwareConfiguration.kLabelCameraAIFront);
        cameraAprilTag = opMode.hardwareMap.get(WebcamName.class, RobotConstants.HardwareConfiguration.kLabelCameraAprilTag);
        cameraStream = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, RobotConstants.HardwareConfiguration.kLabelCameraAprilTag));

        // Sensors
//        sensorAllianceTag = opMode.hardwareMap.get(ColorSensor.class, RobotConstants.HardwareConfiguration.kLabelSensorAllianceTag);

        // Display Telemetry
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.addData(">", "-- AI Camera                      --");
        opMode.telemetry.addData(">", "------------------------------------");

        // Configure Sensor(s)
//        sensorAllianceTag.enableLed(RobotConstants.Sensors.AllianceTag.kIsLedEnabled);

        // Configure Camera(s)
        if(!cameraAIFront.knock()) {
            opMode.telemetry.addData(">", " ERROR: Cannot communicate with " + cameraAIFront.getDeviceName());
        }
        else {
            opMode.telemetry.addData(">", " AI Camera Initialized");
        }

        // Initialize in April Tag Mode
        setAICameraMode(RobotConstants.Vision.HuskyLens.kLabelCameraModeAprilTag);

//        opMode.telemetry.addData("Alliance Color", getDetectedAllianceTagColor());


        // Display Telemetry
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.addData(">", "-- April Tag Camera               --");
        opMode.telemetry.addData(">", "------------------------------------");

//        processorAprilTag = AprilTagProcessor.easyCreateWithDefaults();
        processorAprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(positionCameraAprilTag, orientationCameraAprilTag)
                .build();

        VisionPortal.Builder visionBuilder = new VisionPortal.Builder();

        visionBuilder.setCamera(cameraAprilTag);

        // Choose a camera resolution. Not all cameras support all resolutions.
        visionBuilder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        visionBuilder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        visionBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        visionBuilder.setAutoStopLiveView(false);

        // Set and enable the processor.
        visionBuilder.addProcessor(processorAprilTag);

        // Build the Vision Portal, using the above settings.
//        visionPortalAprilTag = VisionPortal.easyCreateWithDefaults(cameraAprilTag, processorAprilTag);
        visionPortalAprilTag = visionBuilder.build();


        // FTC Dashboard
        FtcDashboard.getInstance().startCameraStream(cameraStream, 0);

        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.addData(">", " System: Vision Initialized");
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.update();

    }

    // -----------------------------------------
    // Action Method(s)
    // -----------------------------------------

    /**
     * Add telemetry about AprilTag detections.
     */
    public void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = getListDetectedAprilTags();
        opMode.telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                opMode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    opMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    opMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            } else {
                opMode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                opMode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        opMode.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        opMode.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }   // end method telemetryAprilTag()


    // -----------------------------------------
    // Get Method(s)
    // -----------------------------------------

    public HuskyLens.Block[] getListAICameraObject() {

        HuskyLens.Block[] listBlocks = cameraAIFront.blocks();

        return listBlocks;
    }

    public HuskyLens.Block getAICameraObject(HuskyLens.Block[] listBlocks, int blockID) {
        HuskyLens.Block block = null;

        for (HuskyLens.Block itemBlock: listBlocks) {
            if(itemBlock.id == blockID) {
                block = itemBlock;
            }
        }

        return block;
    }

    public HuskyLens.Block getAICameraObject(HuskyLens.Block[] listBlocks) {
        HuskyLens.Block block = null;

        for (HuskyLens.Block itemBlock: listBlocks) {
            block = itemBlock;
        }

        return block;
    }

    public List<AprilTagDetection> getListDetectedAprilTags() {
        return processorAprilTag.getDetections();
    }

    public String getDetectedAprilTagIds() {
        StringBuilder detectedAprilTagIds = new StringBuilder();

        for (AprilTagDetection detection : getListDetectedAprilTags()) {
            detectedAprilTagIds.append(detection.id);
            detectedAprilTagIds.append(' ');
        }

        return detectedAprilTagIds.toString();
    }

    public AprilTagDetection getDetectedLocalization() {
        AprilTagDetection localization = null;

        for (AprilTagDetection detection : getListDetectedAprilTags()) {
            if(!detection.metadata.name.contains("Obelisk")) {
                localization = detection;
            }
        }

        return localization;
    }

    public int getDetectedObeliskId() {
        int detectedObeliskId = 0;

        for (AprilTagDetection detection : getListDetectedAprilTags()) {
            if(detection.metadata.name.contains("Obelisk")) {
                detectedObeliskId = detection.id;
            }
        }

        return detectedObeliskId;
    }

    public Triple<String, String, String> getObeliskPattern(int patternId) {
        Triple<String, String, String> pattern = new Triple<>("NA", "NA", "NA");

        switch (patternId) {

            case RobotConstants.GameElements.AprilTag.Obelisk.kMotifOneId:
                pattern = RobotConstants.GameElements.AprilTag.Obelisk.kMotifOnePattern;
                break;

            case RobotConstants.GameElements.AprilTag.Obelisk.kMotifTwoId:
                pattern = RobotConstants.GameElements.AprilTag.Obelisk.kMotifTwoPattern;
                break;

            case RobotConstants.GameElements.AprilTag.Obelisk.kMotifThreeId:
                pattern = RobotConstants.GameElements.AprilTag.Obelisk.kMotifThreePattern;
                break;

        }

        return pattern;
    }

    public String getDetectedAllianceTagColor() {
        String detectedColor;

        if(sensorAllianceTag.blue() > sensorAllianceTag.red()) {
            detectedColor = "blue";
        }
        else {
            detectedColor = "red";
        }

        return detectedColor;
    }

    public int getAllianceTagColorLevel(String checkColor) {
        int colorLevel;

        switch(checkColor) {
            case("blue"):
                colorLevel = sensorAllianceTag.blue();
                break;
            case("red"):
                colorLevel = sensorAllianceTag.red();
                break;
            case("green"):
                colorLevel = sensorAllianceTag.green();
                break;
            default:
                colorLevel = 0;
        }

        return colorLevel;
    }

    // -----------------------------------------
    // Set Method(s)
    // -----------------------------------------

    public void setAICameraMode(String cameraMode) {

        switch(cameraMode) {
            case(RobotConstants.Vision.HuskyLens.kLabelCameraModeObjectRecognition):
                cameraAIFront.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);
                break;
            case(RobotConstants.Vision.HuskyLens.kLabelCameraModeObjectTracking):
                cameraAIFront.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
                break;
            default:
                cameraAIFront.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
                break;
        }

    }

}
