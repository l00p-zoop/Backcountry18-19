/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Depot Auto", group = "Concept")
@Disabled
public class Depot_Auto extends LinearOpMode {
    //Motor Initialization
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AUKN4dL/////AAABmZL4EHQ+ukIzpu89viYzaR4/jtXdbGjrssYSj0jYOGUszrxyjIZgyw6bnlbe3u5cMKxWpBP4i8HPWkAKoqmaJmRHrv8Ho7vPazLYJ65ZoLevtugk9OihzarMGR1FSjcThK3Tx3k2zpgCwaZWGopNS4ObXr5mFqdZkLmE11hEj7QV/sX886rFKB6Q9ySF1L4eYhwZmsArJjf5BGgenPwg0/Ou8yhB3VXk5++EAbLQwwlSFYZnQkS8h9TeGHe7LL5CbPgYptx+qo8iLJqqx6cqkabRHyVv1rzTxeJw42wFpHAono6FcMrWOiuKZs2xBiKKlUlw2qI8cl6sHbhiRPcOtLO3X5p7f0+fInRqOY4pakdS";
//Vuforia and Tensorflow initialization
    private ElapsedTime runtime = new ElapsedTime();


    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor


    private static final CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.2;
    static final double     TURN_SPEED              = 0.2;

    private DistanceSensor sensorRange;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    DcMotor motor1, motor2, motor3, motor4, liftmotor, rotatemotor, linac;
    Servo hookservo;
//Establishing Constants
    boolean chkloop = true;
    boolean drploop = true;
    boolean timecheck = true;
    double inchCount = 21.5;
    double tickCount = 0;
    double doubleCount = 0;
    double revCount = 0;
    double movetimem = 0;
    double movetimes = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        //Motor Classification
        motor1 = hardwareMap.get(DcMotor.class, "m1");
        motor2 = hardwareMap.get(DcMotor.class, "m2");
        motor3 = hardwareMap.get(DcMotor.class, "m3");
        motor4 = hardwareMap.get(DcMotor.class, "m4");
        liftmotor = hardwareMap.get(DcMotor.class, "lm");
        rotatemotor = hardwareMap.get(DcMotor.class, "rm");
        linac = hardwareMap.get(DcMotor.class, "lc");
        sensorRange = hardwareMap.get(DistanceSensor.class, "dst");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;

        hookservo = hardwareMap.get(Servo.class, "hs");

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //More TensorFlow Setup
        telemetry.addData("Path0", "Starting at %7d :%7d",
                motor1.getCurrentPosition(),
                motor4.getCurrentPosition());
        telemetry.update();

        hookservo.setPosition(.35);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;


        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        sleep(2000);
//autonomous start
        //Robot drops, relying on a distance sensor mounted on the bottom.
        liftmotor.setPower(.2);
        // motor2.setPower(.15);
        while (drploop) {
            if (sensorRange.getDistance(DistanceUnit.CM) < 4.7 && sensorRange.getDistance(DistanceUnit.CM) > 4.3) {
                //continue dropping until the wheels are touching the ground
                drploop = false;
            }
        }
        //Let the lift continue to raise just enough to the point where the whole weight of the robot is resting on the wheels

        sleep(2000);
        liftmotor.setPower(0);//stop the lift
        //Disengage hook
        hookservo.setPosition(-.1);

        motor1.setPower(-.25);
        motor4.setPower(.25);
        sleep(300);//strafe out from the lander just a tiny bit
        motor1.setPower(0);
        motor4.setPower(0);

        //Move forwards to gain distance from the lander and to align with the particles
        motor3.setPower(-.3);
        motor2.setPower(.3);
        sleep(575);
        motor3.setPower(0);
        motor2.setPower(0);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        runtime.reset();
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            while (chkloop) {
                if (opModeIsActive()) {
                    if (tfod != null) {
                        //make sure that TensorFlow Object Detection exists
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        int goldMineralX = 0;
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            //if TFOD cannot find a gold mineral, set goldMineralX to -1
                            goldMineralX = -1;
                            //finds the location of the gold mineral in regards to the camera
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) ((recognition.getLeft() + recognition.getRight()) / 2);
                                }
                            }
                            //Algorithm to align the robot with the gold mineral.
                            telemetry.addData("GoldPos:", goldMineralX);
                            if (goldMineralX == -1 && runtime.seconds() < 4) {
                                //If the robot doesn't see the gold mineral it moves left for 4 seconds
                                motor1.setPower(.25);
                                motor4.setPower(-.25);
                            } else if (goldMineralX == -1 && runtime.seconds() >= 4 && runtime.seconds() < 9.5) {
                                //If the motor still doesn't see the mineral after 4 seconds, it changes direction
                                motor1.setPower(-.25);
                                motor4.setPower(.25);
                            } else if(goldMineralX == -1){
                                //If it still doesn't see the cube after 10 seconds it breaks out of the loop.
                                movetimem = runtime.milliseconds();
                                movetimes = runtime.seconds();
                                motor1.setPower(0);
                                motor4.setPower(0);
                                chkloop = false;

                            } else if (goldMineralX > 200 && goldMineralX < 700) {
                                //If it sees the mineral and it's within an acceptable range, it moves onto the next part of the program.
                                //It also saves how long it took to find the gold to be used later.
                                movetimem = runtime.milliseconds();
                                movetimes = runtime.seconds();
                                motor1.setPower(0);
                                motor4.setPower(0);
                                chkloop = false;

                            }

                        }
                        telemetry.update();


                    }
                }
            }

             telemetry.addData("Milliseconds:",movetimem);
             telemetry.addData("Seconds:",movetimes);
             telemetry.update();

            //The robot moves forwards and backwards to push the mineral off of its position.
            motor3.setPower(-.5);
            motor2.setPower(.5);
            sleep(1500);
            motor3.setPower(.5);
            motor2.setPower(-.5);
            sleep(1200);
            motor3.setPower(0);
            motor2.setPower(0);
            //Algorithm to move the robot to a given location, based on how long it's been driving.
            if (movetimes <= 4){
                //If the the robot was moving for less than four seconds, we move it left this amount to get to our desired position
                motor1.setPower(.25);
                motor4.setPower(-.25);
                sleep((int)(4000-movetimem));
                motor1.setPower(0);
                motor4.setPower(0);
            } else if (movetimes > 4){
                //Otherwise, the robot has doubled back, so we move it left for the amount of time it has moved, minus 4
                motor1.setPower(.25);
                motor4.setPower(-.25);
                sleep((int)(movetimem - 4000));
                motor1.setPower(0);
                motor4.setPower(0);
            }
            //Here, the robot rotates so that it is facing the depot
            motor1.setPower(-.25);
            motor2.setPower(-.25);
            motor3.setPower(-.25);
            motor4.setPower(-.25);
            sleep(500);
            motor1.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);
            motor4.setPower(0);
            // Robot moves left to reach the wall
            motor1.setPower(.4);
            motor4.setPower(-.4);
            sleep(1600);
            motor1.setPower(0);
            motor4.setPower(0);
            //Robot moves forward into the depot.
            motor3.setPower(-1);
            motor2.setPower(1);
            sleep(1600);
            motor3.setPower(0);
            motor2.setPower(0);
            //Robot arm rotates to dislodge the team marker.
            rotatemotor.setPower(-.8);
            sleep(1000);
            rotatemotor.setPower(0);


            if (tfod != null) {
                tfod.shutdown();
            }

        }

    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    public void FLBRDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motor1.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = motor4.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            motor1.setTargetPosition(newLeftTarget);
            motor4.setTargetPosition(newRightTarget);


            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            motor1.setPower(Math.abs(speed));
            motor4.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motor1.isBusy() && motor4.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        motor1.getCurrentPosition(),
                        motor4.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motor1.setPower(0);
            motor4.setPower(0);

            // Turn off RUN_TO_POSITION
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

        }
    }//Function to make motor programming easier
    //Unfortunately, our motor encoders failed to work for a multitude of reasons, so these are unused
    public void BLFRDrive(double speed,
                          double leftInches, double rightInches,
                          double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motor2.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = motor3.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            motor2.setTargetPosition(newLeftTarget);
            motor3.setTargetPosition(newRightTarget);

            //Move motors to position
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            motor2.setPower(Math.abs(speed));
            motor3.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motor2.isBusy() && motor3.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        motor2.getCurrentPosition(),
                        motor3.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motor2.setPower(0);
            motor3.setPower(0);

            // Turn off RUN_TO_POSITION
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

        }
    }
}
