package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="RR_RightAuto", group="Pushbot")
public class RR_RightAuto extends LinearOpMode
{
    //INTRODUCE VARIABLES HERE

    //create motors here so they are accessible throughout the op mode
    DcMotor armLiftLeft;
    DcMotor armLiftRight;
    DcMotor waiter;
    Servo closerL;
    Servo extender;

    double     ARM_SPEED_RISING       = 0.42;
    double     ARM_SPEED_LOWER        = 0.42;

    //values to manage changes in lift height as it picks up cones from the stack
    int coneHeightAdjustment = 0; //starts at zero and then increments using increment constant
    int coneHeightIncrement = 150; //counts for lift motors

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
        //connect to drivetrain motors and encoders
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //set up non-drivetrain motors
        armLiftLeft = hardwareMap.get(DcMotor.class, "armLiftLeft");
        armLiftRight = hardwareMap.get(DcMotor.class, "armLiftRight");
        waiter = hardwareMap.get(DcMotor.class, "waiter");
        closerL = hardwareMap.get(Servo.class, "closerL");
        extender = hardwareMap.get(Servo.class, "extender");

        //set directions
        armLiftLeft.setDirection(DcMotor.Direction.REVERSE);
        armLiftRight.setDirection(DcMotor.Direction.FORWARD);
        waiter.setDirection(DcMotor.Direction.REVERSE);

        //encoder setup
        armLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waiter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waiter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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

        //roadrunner trajectory building - get called after wait for start

        //prepare trajectory for driving to spot
        Trajectory driveToSpot = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 0), 0)
                .build();

        //prepare left and right parking trajectories
        Trajectory parkLeft = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(24)
                .build();

        Trajectory parkRight = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(24)
                .build();


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
                telemetry.addLine("Null: middle parking spot");
                telemetry.update();

                //clamp preloaded cone
                clampCone();

                //drive to spot
                drive.followTrajectory(driveToSpot);

                //place cones
                placeCones();

                //park middle: stay

            }else if(tagOfInterest.id == LEFT){
                //left parking spot
                telemetry.addLine("Left parking spot");
                telemetry.update();

                //clamp preloaded cone
                clampCone();

                //drive to spot
                drive.followTrajectory(driveToSpot);

                //cone placement sequence
                placeCones();

                //park left
                drive.followTrajectory(parkLeft);

            }else if(tagOfInterest.id == MIDDLE){
                //middle parking spot
                telemetry.addLine("Middle parking spot");
                telemetry.update();

                //clamp preloaded cone
                clampCone();

                //drive to spot
                drive.followTrajectory(driveToSpot);

                //place cones
                placeCones();

                //park middle: stay

            }else{
                //right parking spot
                sleep(2000);
                telemetry.addLine("Right parking spot");
                telemetry.update();

                //clamp preloaded cone
                clampCone();

                //drive to spot
                drive.followTrajectory(driveToSpot);

                //place cones
                placeCones();

                //park right
                drive.followTrajectory(parkRight);

            }

            //stop robot
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


    /* MOVEMENT FUNCTIONS */

    public void placeCones() {
        //place preloaded cone
        raise(3000);
        rotateWaiter(250);
        raise(-200);
        releaseCone();

        //pick up and place second cone
        autoConeCycle();

        //pick up and place third cone
        autoConeCycle();

    }

    public void clampCone() {

        closerL.setPosition(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addLine("clamp cone");
            telemetry.update();
        }

    }

    public void releaseCone() {
        closerL.setPosition(.5); // open claw
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addLine("release cone");
            telemetry.update();
        }
    }

    public void extendClaw() {
        extender.setPosition(1); // open claw
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addLine("extend claw");
            telemetry.update();
        }
    }

    public void retractClaw() {
        extender.setPosition(0); // open claw
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addLine("retract claw");
            telemetry.update();
        }
    }

    public void autoConeCycle(){
        //starts after releasing previous cone
        raise(-2600 - coneHeightAdjustment); //lowers lift
        rotateWaiter(-250); //rotates toward cone stack
        extendClaw();
        clampCone();
        raise(400); //lift cone above stack so it doesn't knock it over when arm retracts
        retractClaw();
        raise(2200 + coneHeightAdjustment); //2200 because 2600-400
        rotateWaiter(250);
        raise(-200); //lower cone onto junction
        releaseCone();
        coneHeightAdjustment += coneHeightIncrement; //increment the coneHeightAdjustment for the next cycle
    }

    public void rotateWaiter(double count) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = waiter.getCurrentPosition() + (int)(count);
            waiter.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            waiter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            waiter.setPower(0.5);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && waiter.isBusy()) {
                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        waiter.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            waiter.setPower(0);

            // Turn off RUN_TO_POSITION
            waiter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }


    //RAISE ARM FUNCTION
    public void raise(double count) {

        int newArmLiftLeftTarget;
        int newArmLiftRightTarget;

        // Determine new target position, and pass to motor controller
        newArmLiftLeftTarget = armLiftLeft.getCurrentPosition() - (int) (count);
        newArmLiftRightTarget = armLiftRight.getCurrentPosition() - (int) (count);
        armLiftLeft.setTargetPosition(newArmLiftLeftTarget);
        armLiftRight.setTargetPosition(newArmLiftRightTarget);

        // Turn On RUN_TO_POSITION
        armLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armLiftLeft.setPower(Math.abs(ARM_SPEED_RISING));
        armLiftRight.setPower(Math.abs(ARM_SPEED_RISING));
        runtime.reset();
        while (opModeIsActive() && (armLiftLeft.isBusy() || armLiftRight.isBusy())) {
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newArmLiftLeftTarget, newArmLiftRightTarget);
            telemetry.update();
        }

        // Stop all motion
        armLiftRight.setPower(0);
        armLiftLeft.setPower(0);

        //Turn off RUN_TO_POSITION
//        robot.armLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.armLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //LOWER ARM FUNCTION
    public void lower(double count) {

        int newArmLiftRightTarget;
        int newArmLiftLeftTarget;

        // Determine new target position, and pass to motor controller
        newArmLiftRightTarget = armLiftRight.getCurrentPosition() + (int) (count);
        newArmLiftLeftTarget = armLiftLeft.getCurrentPosition() + (int) (count);
        armLiftRight.setTargetPosition(newArmLiftRightTarget);
        armLiftLeft.setTargetPosition(newArmLiftLeftTarget);

        // Turn On RUN_TO_POSITION
        armLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armLiftRight.setPower(Math.abs(ARM_SPEED_LOWER));
        armLiftLeft.setPower(Math.abs(ARM_SPEED_LOWER));
        runtime.reset();
        while (opModeIsActive() && (armLiftLeft.isBusy() || armLiftRight.isBusy())) {
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newArmLiftLeftTarget, newArmLiftRightTarget);
            telemetry.update();
        }

        // Stop all motion
        armLiftRight.setPower(0);
        armLiftLeft.setPower(0);
//
//        // Turn off RUN_TO_POSITION
//        robot.armLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.armLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}