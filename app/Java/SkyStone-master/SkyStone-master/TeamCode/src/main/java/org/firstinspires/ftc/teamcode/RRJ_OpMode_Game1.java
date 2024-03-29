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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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

@TeleOp(name="RRJ_OpMode_Game1", group="Linear Opmode")
//@Disabled
public class RRJ_OpMode_Game1 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo leftClawServo = null;
    private Servo rightClawServo = null;
    private double servoPosition = 0.0;
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");

        leftClawServo = hardwareMap.get(Servo.class, "LeftClaw");
        rightClawServo = hardwareMap.get(Servo.class, "RightClaw");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftClawServo.setDirection(Servo.Direction.FORWARD);
        rightClawServo.setDirection(Servo.Direction.REVERSE);

        double  position = 1.0; //(MAX_POS - MIN_POS) / 2; // Start at halfway position
        boolean rampUp = true;
        if (position != 1.0)
        {
            leftClawServo.setPosition(position);
            rightClawServo.setPosition(position);
        }
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            //double armLifter = gamepad1.dpad_up();
            //double armDown = gamepad1.dpad_down();


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double driveLeft = gamepad1.left_stick_y;
            double driveRight = gamepad1.right_stick_y;
            double turn  =  gamepad1.right_stick_x;
//            leftPower    = Range.clip(drive - turn, -1.0, 1.0) ;
//            rightPower   = Range.clip(drive + turn, -1.0, 1.0) ;

            if (driveLeft<0) {
                leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
                // Send calculated power to wheels
                leftFrontDrive.setPower(-1.0);
                leftBackDrive.setPower(-1.0);
            }
            else if (driveLeft>0) {
                leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
                // Send calculated power to wheels
                leftFrontDrive.setPower(1.0);
                leftBackDrive.setPower(1.0);
            }
            if (driveRight<0) {
                rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
                // Send calculated power to wheels
                rightFrontDrive.setPower(-1.0);
                rightBackDrive.setPower(-1.0);
            }
            else if (driveRight>0) {
                rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
                // Send calculated power to wheels
                rightFrontDrive.setPower(1.0);
                rightBackDrive.setPower(1.0);
            }
//            else if (drive>0) {
//                leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//                rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//                leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//                rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//                // Send calculated power to wheels
//                leftFrontDrive.setPower(drive);
//                rightFrontDrive.setPower(drive);
//                leftBackDrive.setPower(drive);
//                rightBackDrive.setPower(drive);
//            }


            if (turn<0)
            {
                // Reverse the motor that runs backwards when connected directly to the battery
                leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
                rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
                // Send calculated power to wheels
                leftFrontDrive.setPower(turn);
                rightFrontDrive.setPower(turn);
                leftBackDrive.setPower(turn);
                rightBackDrive.setPower(turn);

            }
                else if(turn>0){
                leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
                rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
                // Send calculated power to wheels
                leftFrontDrive.setPower(turn);
                rightFrontDrive.setPower(turn);
                leftBackDrive.setPower(turn);
                rightBackDrive.setPower(turn);

            }


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftFrontDrive.setPower(0.0);
            rightFrontDrive.setPower(0.0);
            leftBackDrive.setPower(0.0);
            rightBackDrive.setPower(0.0);




            // slew the servo, according to the rampUp (direction) variable.
            if (position!=MAX_POS && gamepad1.y) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT ;
//                if (position >= MAX_POS ) {
//                    position = MAX_POS;
//                    rampUp = !rampUp;   // Switch ramp direction
//                }
            }
            else if (position != MIN_POS && gamepad1.a) {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
//                if (position <= MIN_POS ) {
//                    position = MIN_POS;
//                    rampUp = !rampUp;  // Switch ramp direction
//                }
            }
            // Set the servo to the new position and pause;
            leftClawServo.setPosition(position);
            rightClawServo.setPosition(position);
            sleep(CYCLE_MS);
            //idle();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Gamepad1 button pressed", gamepad1.getRobocolMsgType());
            telemetry.addData("ServoMotors", "left (%.2f), right (%.2f)", leftClawServo.getPosition(), rightClawServo.getPosition());
            telemetry.update();

        }
    }
}
