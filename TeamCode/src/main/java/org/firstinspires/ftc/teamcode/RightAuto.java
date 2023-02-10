package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



import java.util.ArrayList;

@Autonomous(name="RightAuto", group="Pushbot")
public class RightAuto extends LinearOpMode
{
    //INTRODUCE VARIABLES HERE

    public robotInit robot = new robotInit();
    ElapsedTime runtime = new ElapsedTime();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    /*EDIT IF NEEDED!!!*/

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();



        while (opModeIsActive())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    sleep(2000);

                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);

            if(tagOfInterest != null)
            {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
                sleep(2000);

            }
            else
            {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

            //PUT AUTON CODE HERE (DRIVER PRESSED THE PLAY BUTTON!)


            /* Actually do something useful */
            if(tagOfInterest == null){
                //default trajectory here if preferred
                sleep(2000);
                telemetry.addLine("Null: middle trajectory");
                telemetry.update();

                clampCone();
                raise(500);
                moveForward(65);
                raise(2500);
                moveLeft(5);
                raise(-200);
                releaseCone();
//        runtime.reset();
//        while (opModeIsActive() && runtime.seconds() < 2.0) {
//            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
                moveRight(5);
                lower(2500);
                moveBackward(12);
                //park middle
                //stay


            }else if(tagOfInterest.id == LEFT){
                //left trajectory
                sleep(2000);
                telemetry.addLine("Left trajectory");
                telemetry.update();

                //park left
                clampCone();
                raise(500);
                moveForward(65);
                raise(2500);
                moveLeft(5);
                raise(-200);
                releaseCone();
//        runtime.reset();
//        while (opModeIsActive() && runtime.seconds() < 2.0) {
//            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
                moveRight(5);
                lower(2500);
                moveBackward(12);
                moveLeft(30);


            }else if(tagOfInterest.id == MIDDLE){
                //middle trajectory
                sleep(2000);
                telemetry.addLine("Middle trajectory");
                telemetry.update();

                //park middle
                //stay
                clampCone();
                raise(500);
                moveForward(65);
                raise(2500);
                moveLeft(5);
                raise(-200);
                releaseCone();
//        runtime.reset();
//        while (opModeIsActive() && runtime.seconds() < 2.0) {
//            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
                moveRight(5);
                lower(2500);
                moveBackward(12);

            }else{
                //right trajectory
                sleep(2000);
                telemetry.addLine("Right trajectory");
                telemetry.update();

                //park right
                clampCone();
                raise(500);
                moveForward(65);
                raise(2500);
                moveLeft(5);
                raise(-200);
                releaseCone();
//        runtime.reset();
//        while (opModeIsActive() && runtime.seconds() < 2.0) {
//            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
                moveRight(5);
                lower(2500);
                moveBackward(12);
                moveRight(30);

            }

            //stop robot
            sleep(1000);     // pause for servos to move

            telemetry.addData("Path", "Complete");
            telemetry.update();
            stop();

        }
    }


    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    /* ENCODER FUNCTIONS */
    public void resetEncoder()
    {
        robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void startEncoderMode()
    {
        //Set Encoder Mode
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /* MOVEMENT FUNCTIONS */

    public void placeCones() {
        clampCone();
        raise(500);
        moveForward(66);
        raise(2500);
        moveLeft(5);
        raise(-200);
        releaseCone();
//        runtime.reset();
//        while (opModeIsActive() && runtime.seconds() < 2.0) {
//            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
        moveRight(10);
        lower(2500);
        moveBackward(15);
    }

    public void clampCone() {

        robot.closerL.setPosition(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

    }

    public void releaseCone() {
        robot.closerL.setPosition(.5); // open claw
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }



    public void moveForward(double inches) {
        int newmotorFLTarget;
        int newmotorFRTarget;
        int newmotorBLTarget;
        int newmotorBRTarget;

        // Determine new target position, and pass to motor controller
        newmotorFLTarget = robot.motorFL.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
        newmotorFRTarget = robot.motorFR.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
        newmotorBLTarget = robot.motorBL.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
        newmotorBRTarget = robot.motorBR.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
        robot.motorFL.setTargetPosition(newmotorFLTarget);
        robot.motorFR.setTargetPosition(newmotorFRTarget);
        robot.motorBL.setTargetPosition(newmotorBLTarget);
        robot.motorBR.setTargetPosition(newmotorBRTarget);

        // Turn On RUN_TO_POSITION
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
        runtime.reset();
        while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget );
            telemetry.update();
        }

        // Stop all motion;
        stopRobot();

        // Turn off RUN_TO_POSITION
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void moveBackward(double inches) {
        int newmotorFLTarget;
        int newmotorFRTarget;
        int newmotorBLTarget;
        int newmotorBRTarget;

        // Determine new target position, and pass to motor controller
        newmotorFLTarget = robot.motorFL.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
        newmotorFRTarget = robot.motorFR.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
        newmotorBLTarget = robot.motorBL.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
        newmotorBRTarget = robot.motorBR.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
        robot.motorFL.setTargetPosition(newmotorFLTarget);
        robot.motorFR.setTargetPosition(newmotorFRTarget);
        robot.motorBL.setTargetPosition(newmotorBLTarget);
        robot.motorBR.setTargetPosition(newmotorBRTarget);

        // Turn On RUN_TO_POSITION
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));

        runtime.reset();
        while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget );
            telemetry.update();
        }

        // Stop all motion;
        stopRobot();

        // Turn off RUN_TO_POSITION
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    public void moveRight(double inches) {
        int newmotorFLTarget;
        int newmotorFRTarget;
        int newmotorBLTarget;
        int newmotorBRTarget;

        // Determine new target position, and pass to motor controller
        newmotorFLTarget = robot.motorFL.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
        newmotorFRTarget = robot.motorFR.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
        newmotorBLTarget = robot.motorBL.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
        newmotorBRTarget = robot.motorBR.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
        robot.motorFL.setTargetPosition(newmotorFLTarget);
        robot.motorFR.setTargetPosition(newmotorFRTarget);
        robot.motorBL.setTargetPosition(newmotorBLTarget);
        robot.motorBR.setTargetPosition(newmotorBRTarget);

        // Turn On RUN_TO_POSITION
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));

        runtime.reset();
        while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget );
            telemetry.update();
        }

        // Stop all motion;
        stopRobot();

        // Turn off RUN_TO_POSITION
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void moveLeft(double inches) {
        int newmotorFLTarget;
        int newmotorFRTarget;
        int newmotorBLTarget;
        int newmotorBRTarget;

        // Determine new target position, and pass to motor controller
        newmotorFLTarget = robot.motorFL.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
        newmotorFRTarget = robot.motorFR.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
        newmotorBLTarget = robot.motorBL.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
        newmotorBRTarget = robot.motorBR.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
        robot.motorFL.setTargetPosition(newmotorFLTarget);
        robot.motorFR.setTargetPosition(newmotorFRTarget);
        robot.motorBR.setTargetPosition(newmotorBRTarget);

        // Turn On RUN_TO_POSITION
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));

        runtime.reset();
        while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
            telemetry.update();
        }

        // Stop all motion;
        stopRobot();

        // Turn off RUN_TO_POSITION
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void turnleft(double inches) {
        int newmotorFLTarget;
        int newmotorFRTarget;
        int newmotorBLTarget;
        int newmotorBRTarget;

        // Determine new target position, and pass to motor controller
        newmotorFLTarget = robot.motorFL.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);      newmotorFRTarget = robot.motorFR.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
        newmotorBLTarget = robot.motorBL.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);       newmotorBRTarget = robot.motorBR.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);

        robot.motorFL.setTargetPosition(newmotorFLTarget);
        robot.motorFR.setTargetPosition(newmotorFRTarget);
        robot.motorBL.setTargetPosition(newmotorBLTarget);
        robot.motorBR.setTargetPosition(newmotorBRTarget);

        // Turn On RUN_TO_POSITION
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
        runtime.reset();
        while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget );
            telemetry.update();
        }

        // Stop all motion;
        stopRobot();

        // Turn off RUN_TO_POSITION
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnright(double inches) {
        int newmotorFLTarget;
        int newmotorFRTarget;
        int newmotorBLTarget;
        int newmotorBRTarget;

        // Determine new target position, and pass to motor controller
        newmotorFLTarget = robot.motorFL.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);      newmotorFRTarget = robot.motorFR.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
        newmotorBLTarget = robot.motorBL.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);      newmotorBRTarget = robot.motorBR.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
        robot.motorFL.setTargetPosition(newmotorFLTarget);
        robot.motorFR.setTargetPosition(newmotorFRTarget);
        robot.motorBL.setTargetPosition(newmotorBLTarget);
        robot.motorBR.setTargetPosition(newmotorBRTarget);

        // Turn On RUN_TO_POSITION
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
        runtime.reset();
        while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget );
            telemetry.update();
        }

        // Stop all motion;
        stopRobot();

        // Turn off RUN_TO_POSITION
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //RAISE ARM FUNCTION
    public void raise(double count) {

        int newArmLiftLeftTarget;
        int newArmLiftRightTarget;

        // Determine new target position, and pass to motor controller
        newArmLiftLeftTarget = robot.armLiftLeft.getCurrentPosition() - (int) (count);
        newArmLiftRightTarget = robot.armLiftRight.getCurrentPosition() - (int) (count);
        robot.armLiftLeft.setTargetPosition(newArmLiftLeftTarget);
        robot.armLiftRight.setTargetPosition(newArmLiftRightTarget);

        // Turn On RUN_TO_POSITION
        robot.armLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.armLiftLeft.setPower(Math.abs(robot.ARM_SPEED_RISING));
        robot.armLiftRight.setPower(Math.abs(robot.ARM_SPEED_RISING));
        runtime.reset();
        while (opModeIsActive() && (robot.armLiftLeft.isBusy() || robot.armLiftRight.isBusy())) {
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newArmLiftLeftTarget, newArmLiftRightTarget);
            telemetry.update();
        }

        //TRIAL 1: Both commented, arm stops before moveLeft
        //TRIAL 2: Commenting stopRobot() and moveLeft, arm keeps rising and doesn't do later code
        //TRIAL 3: Comment out stop, encoder, and moveLeft
        //TRIAL 4: comment out encoder, works! (the arm raises and stops raising)
        //TRIAL 5: uncomment raise() after moveForward(), doesn't raise the arm at the specified height
        //TRIAL 6:


        // Stop all motion;
        stopRobot();

        //Turn off RUN_TO_POSITION
//        robot.armLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.armLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //LOWER ARM FUNCTION
    public void lower(double count) {

        int newArmLiftRightTarget;
        int newArmLiftLeftTarget;

        // Determine new target position, and pass to motor controller
        newArmLiftRightTarget = robot.armLiftRight.getCurrentPosition() + (int) (count);
        newArmLiftLeftTarget = robot.armLiftLeft.getCurrentPosition() + (int) (count);
        robot.armLiftRight.setTargetPosition(newArmLiftRightTarget);
        robot.armLiftLeft.setTargetPosition(newArmLiftLeftTarget);

        // Turn On RUN_TO_POSITION
        robot.armLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.armLiftRight.setPower(Math.abs(robot.ARM_SPEED_LOWER));
        robot.armLiftLeft.setPower(Math.abs(robot.ARM_SPEED_LOWER));
        runtime.reset();
        while (opModeIsActive() && (robot.armLiftLeft.isBusy() || robot.armLiftRight.isBusy())) {
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newArmLiftLeftTarget, newArmLiftRightTarget);
            telemetry.update();
        }

        // Stop all motion;
        stopRobot();
//
//        // Turn off RUN_TO_POSITION
//        robot.armLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.armLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void stopRobot() {
        robot.motorFL.setPower(0);
        robot.motorFR.setPower(0);
        robot.motorBL.setPower(0);
        robot.motorBR.setPower(0);
    }


//    public void placeCone(){
//        //move motor down
//        robot.armLiftLeft.setPower(0.2);
//        robot.armLiftRight.setPower(0.2);
//        runtime.reset();
//        while (runtime.seconds() < 0.6){
//        }
//
//        //unclamp servo
//        robot.armLiftLeft.setPower(0);
//        robot.armLiftRight.setPower(0);
//        robot.closerL.setPosition(0.5);
//        robot.closerR.setPosition(0);
//
//
//        //wait
//        runtime.reset();
//        while (runtime.seconds() < 1){
//        }
//
//        //move arm back down
//        robot.armLiftRight.setPower(-0.2);
//        robot.armLiftLeft.setPower(-0.2);
//        runtime.reset();
//        while (runtime.seconds() < 0.5){
//        }
//
//        robot.armLiftLeft.setPower(0);
//        robot.armLiftRight.setPower(0);
//    }


    public void stopRobot(int seconds) {
        //delay
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            stopRobot();
            telemetry.addData("Motor", "Stopped");    //
            telemetry.update();
        }
    }
}