//package org.firstinspires.ftc.teamcode.common;
//
////import static org.firstinspires.ftc.teamcode.comp.common.BetaBionixCommon.BLUE_ALLIANCE;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
////import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
////import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//
//import java.util.List;
//import java.util.concurrent.TimeUnit;
//
//public class VisionProcessor {
//
//    private LinearOpMode linearOpMode;
//    private HardwareMap hardwareMap;
//    private static final boolean USE_WEBCAM = true;
//    private static final String BLUE_TFOD_MODEL_ASSET = "BlueProp.tflite";
//    private final String[] BLUE_LABELS = {"BlueProp"};
//    private static final String RED_TFOD_MODEL_ASSET = "RedProp.tflite";
//    private final String[] RED_LABELS = {"RedProp"};
//    private AprilTagProcessor aprilTag;
//    //private TfodProcessor tfod;
//    private VisionPortal visionPortal;
//
//    public VisionProcessor(LinearOpMode callingLinearOpMode, String alliance){
//        this.linearOpMode = callingLinearOpMode;
//        this.hardwareMap = callingLinearOpMode.hardwareMap;
//        initVisionProcessors(alliance);
//    }
//
//    public VisionProcessor(LinearOpMode callingLinearOpMode){
//        this.linearOpMode = callingLinearOpMode;
//        this.hardwareMap = callingLinearOpMode.hardwareMap;
//        initVisionProcessor();
//
//    }
//
//  public void initPropDetection(){
//        List<Recognition> currentRecognitions = tfod.getFreshRecognitions();
//        linearOpMode.sleep(1500);
//
//        //for (Recognition recognition : currentRecognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
//            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//
//            linearOpMode.telemetry.addData("Position", "%.0f / %.0f", x, y);
//            linearOpMode.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            linearOpMode.telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//            linearOpMode.telemetry.update();
//            linearOpMode.sleep(200);
//
//        }   // end for() loop
//    }
//
//    public int  isPropDetected(){
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        linearOpMode.sleep(200);
//
//        // -1 means object left
//        // 0 means object ahead
//        // 1 means object on right
//
//        int iPosition = -1;
//
//        if(currentRecognitions !=null  && currentRecognitions.size() > 0){
//            linearOpMode.telemetry.addData("# Objects Detected", currentRecognitions.size());
//            linearOpMode.telemetry.update();
//
//            // Step through the list of recognitions and display info for each one.
//            for (Recognition recognition : currentRecognitions) {
//                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
//                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//
//                // ahead
//                if (x < 300) {
//                    iPosition = 0;
//                }
//                // right
//                else if (x > 300) {
//                    iPosition = 1;
//                }
//                // left
//                else {
//                    iPosition = -1;
//                }
//
//            }   // end for() loop
//
//            return iPosition;
//        }
//
//        return iPosition;
//    }
//
//
//
//    public AprilTagDetection getAprilTagDetection(int desiredTagId){
//        AprilTagDetection desiredTag  = null;
//
//        // Step through the list of detected tags and look for a matching tag
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        if (currentDetections != null){
//            linearOpMode.telemetry.addData("NumberOfDetections",currentDetections.size());
//            linearOpMode.telemetry.update();
//            for (AprilTagDetection detection : currentDetections) {
//                // Look to see if we have size info on this tag.
//                if (detection.metadata != null) {
//                    //  Check to see if we want to track towards this tag.
//                    if ((desiredTagId < 0) || (detection.id == desiredTagId)) {
//                        // Yes, we want to use this tag.
//                        desiredTag = detection;
//                        break;  // don't look any further.
//                    }
//                }
//            }
//        }
//
//        // Tell the driver what we see, and what to do.
//        if (desiredTag != null) {
//            linearOpMode.telemetry.addData("\n>","HOLD X to Drive to Target\n");
//            linearOpMode.telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
//            linearOpMode.telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
//            linearOpMode.telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
//            linearOpMode.telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
//        } else {
//            linearOpMode.telemetry.addData("\n>","Drive using joysticks to find valid target\n");
//        }
//        linearOpMode.telemetry.update();
//        linearOpMode.sleep(200);
//
//        return desiredTag;
//    }
//
//    private void initVisionProcessor() {
//        aprilTag = new AprilTagProcessor.Builder().build();
//        aprilTag.setDecimation(2);
//
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }
//        builder.addProcessor(aprilTag);
//        builder.enableLiveView(true);
//
//        visionPortal = builder.build();
//    }
//
//    private void initVisionProcessors(String alliance) {
//        aprilTag = new AprilTagProcessor.Builder().build();
//        aprilTag.setDecimation(2);
//
//        if(BLUE_ALLIANCE.equals(alliance)){
//            tfod = new TfodProcessor.Builder()
//                    .setModelFileName(BLUE_TFOD_MODEL_ASSET)
//                    .setModelLabels(BLUE_LABELS)
//                    .build();
//        } else{
//            tfod = new TfodProcessor.Builder()
//                    .setModelFileName(RED_TFOD_MODEL_ASSET)
//                    .setModelLabels(RED_LABELS)
//                    .build();
//        }
//        tfod.setMinResultConfidence(0.95f);
//
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }
//        builder.addProcessor(aprilTag);
//        builder.addProcessor(tfod);
//        builder.enableLiveView(true);
//
//        visionPortal = builder.build();
//    }
//
//    private void setManualExposure(int exposureMS, int gain) {
//        // Wait for the camera to be open, then use the controls
//        if (visionPortal == null) {
//            return;
//        }
//
//        // Make sure camera is streaming before we try to set the exposure controls
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            while (!linearOpMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                linearOpMode.sleep(20);
//            }
//        }
//
//        // Set camera controls unless we are stopping.
//        if (!linearOpMode.isStopRequested())
//        {
//            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//                linearOpMode.sleep(50);
//            }
//            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
//            linearOpMode.sleep(20);
//            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//            gainControl.setGain(gain);
//            linearOpMode.sleep(20);
//        }
//    }
//
//    public void end(){
//        visionPortal.close();
//    }
//}
