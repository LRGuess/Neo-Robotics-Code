/* Copyright (c) 2021 FIRST. All rights reserved.
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

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.GregorianCalendar;


@TeleOp(name="TeleOP", group="Linear Opmode")
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.


    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() {

        double armMotorSpeed = 1.0;
        double wheelsMotorSpeed = 1.0;
        double aimAssistMotorSpeed = 0.1;

        boolean useAimAssist = true;

        DcMotor greenMotor;
        DcMotor blackMotor;
        DcMotor armMotor;

        Servo rightHandServo;
        Servo leftHandServo;

        DigitalChannel centerTouchSensor;

        //region Hardware Mapping and Mode/Direction Setting
        greenMotor = hardwareMap.get (DcMotor.class, "greenMotor");
        blackMotor = hardwareMap.get (DcMotor.class, "blackMotor");
        armMotor = hardwareMap.get (DcMotor.class, "armMotor");
        rightHandServo = hardwareMap.get (Servo.class, "rightHandServo");
        leftHandServo = hardwareMap.get (Servo.class, "leftHandServo");
        centerTouchSensor = hardwareMap.get (DigitalChannel.class, "centerTouchSensor");

        greenMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        centerTouchSensor.setMode(DigitalChannel.Mode.INPUT);
        //endregion

        //region telemetry
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        resetRuntime();
        waitForStart();
        //endregion

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //region AimAssist Warning
            /**===================================================================================/*
            -------------------------------=AIM ASSIST INFO=--------------------------------------/*
            ======================================================================================/*

            Aim Assist is to help you put the cone on the pole without dropping it and with
            maximum precision. This is done by reading the input from some sensors such as
            the touch sensor. By reading the info it will regulate the motor speeds and half
            drive the robot to help you if you do not interfere.

            =======INTERFERING WITH THE AIM ASSIST WILL CAUSE YOU TO LOSE MOTOR POWER AND YOU
                   WILL PROBABLY LOSE THE CONE AND SOME CONTROL FOR THE ROBOT================
             */
            //endregion

            //region Drive code
            greenMotor.setPower(-gamepad1.left_stick_y);
            blackMotor.setPower(-gamepad1.right_stick_y);
            //endregion

            //region Arm Motor Speed
            //Increase the motor speed for the arm by pressing the left bumper gamepad2
            if (gamepad2.left_bumper)
                armMotorSpeed = armMotorSpeed + 0.1;
                //If you try to make the motor speed higher then 1.0, set the speed to 1.0.
                if (armMotorSpeed >= 1.0)
                    armMotorSpeed = 1.0;

            //Decrease the motor speed for the arm by pressing the right bumper on gamepad2
            if (gamepad2.right_bumper)
                armMotorSpeed = armMotorSpeed - 0.1;
                //If you try to make the motor speed lower then 0, set the speed to 0.0.
                if (armMotorSpeed <= 0.0)
                    armMotorSpeed = 0.0;
            //endregion

            //region Raise/Lower Arm
            //Raise the arm when you press the a button on gamepad2
            if (gamepad2.a)
                armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                armMotor.setPower(armMotorSpeed);

            //Lower the arm when you press the b button on gamepad2
            if (gamepad2.b)
                armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                armMotor.setPower(armMotorSpeed);
            //endregion

            if (gamepad2.y)


            //region AimAssist
            //Toggle AimAssist when you press the x button on gamepad1
            if (gamepad1.x)
                if (useAimAssist)
                    useAimAssist = false;
                if (!useAimAssist)
                    useAimAssist = true;

            //AimAssist code
            if (useAimAssist)
                if (!centerTouchSensor.getState())
                    //pressed
                    aimAssistMotorSpeed = 1.0;
                    greenMotor.setPower(aimAssistMotorSpeed);
                    greenMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    blackMotor.setPower(aimAssistMotorSpeed);
                    blackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            //endregion

            // Show the elapsed game time and wheel power
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", greenMotor.getPower(), blackMotor.getPower());
            telemetry.update();
        }
    }}
