/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="BHBot", group="Linear Opmode")
//@Disabled
public class BHBot extends LinearOpMode {

    // Declare OpMode members.
    boolean phonefront = true;
    DcMotor motor1, motor2, motor3, motor4, rotatemotor, extendmotor, linac, liftmotor;
    CRServo servo1, servo2;
    int flip =-1;
    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motor1  = hardwareMap.get(DcMotor.class, "m1");
        motor2 = hardwareMap.get(DcMotor.class, "m2");
        motor3 = hardwareMap.get(DcMotor.class, "m3");
        motor4 = hardwareMap.get(DcMotor.class, "m4");
        rotatemotor = hardwareMap.get(DcMotor.class, "rm");
        extendmotor = hardwareMap.get(DcMotor.class, "em");
        linac = hardwareMap.get(DcMotor.class, "lc");
        liftmotor = hardwareMap.get(DcMotor.class, "lm");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.b && phonefront == true){
                phonefront = false;
                telemetry.addData("Phonefront Mode",1);
                telemetry.update();
                sleep(500);
            } else if (gamepad1.b && phonefront == false){
                phonefront = true;
                telemetry.addData("NumFront Mode", 1);
                telemetry.update();
                sleep(500);
            }

            // Setup a variable for each drive wheel to save power level for telemetry
            if (phonefront) {
                motor2.setPower(Range.clip(gamepad1.left_stick_x - gamepad1.right_stick_x,-1,1));
                motor3.setPower(Range.clip(-gamepad1.left_stick_x - gamepad1.right_stick_x,-1,1));
                motor1.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.right_stick_x,-1,1));
                motor4.setPower(Range.clip(gamepad1.left_stick_y - gamepad1.right_stick_x,-1,1));
            } else if (phonefront == false){
                motor2.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.right_stick_x,-1,1));
                motor3.setPower(Range.clip(gamepad1.left_stick_y - gamepad1.right_stick_x,-1,1));
                motor1.setPower(Range.clip(-gamepad1.left_stick_x - gamepad1.right_stick_x,-1,1));
                motor4.setPower(Range.clip(gamepad1.left_stick_x - gamepad1.right_stick_x,-1,1));
            }

            rotatemotor.setPower(-gamepad2.left_stick_x);
            extendmotor.setPower(-gamepad2.right_stick_y);
            linac.setPower(gamepad2.left_stick_y);
            liftmotor.setPower(-Range.clip(gamepad1.left_trigger, 0 , 1) + Range.clip(gamepad1.right_trigger, 0, 1));



        }
    }
}
