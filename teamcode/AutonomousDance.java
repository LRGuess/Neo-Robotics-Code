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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="Autonomous Dance", group="Autonomous")
public class AutonomousDance extends LinearOpMode {


    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor greenMotor;
        DcMotor blackMotor;
        DcMotor armMotor;

        Servo rightHandServo;
        Servo leftHandServo;

        DigitalChannel centerTouchSensor;

        init();

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


        greenMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
            moveForward(1.0, 5000, greenMotor, blackMotor);
            turnLeft(1.0, 1000, greenMotor, blackMotor);
            stop(greenMotor, blackMotor);

            telemetry.update();
        }


    }
    public void moveForward(double power, int duration, DcMotor _greenMotor, DcMotor _blackMotor) throws InterruptedException {
        _greenMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        _blackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        _greenMotor.setPower(power);
        _blackMotor.setPower(power);

        _greenMotor.setTargetPosition(duration);
        _blackMotor.setTargetPosition(duration);

        _greenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _blackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _greenMotor.setPower(0);
        _blackMotor.setPower(0);
    }

    public void moveBackwards(double power, int duration, DcMotor _greenMotor, DcMotor _blackMotor) throws InterruptedException {
        _greenMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        _blackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        _greenMotor.setPower(power);
        _blackMotor.setPower(power);

        _greenMotor.setTargetPosition(duration);
        _blackMotor.setTargetPosition(duration);

        _greenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _blackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _greenMotor.setPower(0);
        _blackMotor.setPower(0);
    }

    public void turnLeft(double power, int duration, DcMotor _greenMotor, DcMotor _blackMotor){

        _greenMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        _blackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        _greenMotor.setPower(power);
        _blackMotor.setPower(power);

        _greenMotor.setTargetPosition(duration);
        _blackMotor.setTargetPosition(duration);

        _greenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _blackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _greenMotor.setPower(0);
        _blackMotor.setPower(0);
    }

    public void turnRight(double power, int duration, DcMotor _greenMotor, DcMotor _blackMotor) throws InterruptedException {
        _greenMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        _blackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        _greenMotor.setPower(power);
        _blackMotor.setPower(power);

        _greenMotor.setTargetPosition(duration);
        _blackMotor.setTargetPosition(duration);

        _greenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _blackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _greenMotor.setPower(0);
        _blackMotor.setPower(0);
    }

    public void armUp(double power, int duration, DcMotor _armMotor) throws InterruptedException {
        _armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        _armMotor.setPower(power);

        _armMotor.setTargetPosition(duration);

        _armMotor.setPower(0);
    }

    public void armDown(double power, int duration, DcMotor _armMotor) throws InterruptedException {
        _armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        _armMotor.setPower(power);

        _armMotor.setTargetPosition(duration);

        _armMotor.setPower(0);
    }

    public void closeHand(Servo _rightHandServo, Servo _leftHandServo) throws InterruptedException{

    }



    public void stop(DcMotor _greenMotor, DcMotor _blackMotor) {
        _greenMotor.setPower(0);
        _blackMotor.setPower(0);
    }

    public void stopAll(DcMotor _greenMotor, DcMotor _blackMotor, DcMotor _armMotor, Servo _leftHandServo, Servo _rightHandServo){
        _greenMotor.setPower(0);
        _blackMotor.setPower(0);
        _armMotor.setPower(0);
    }

}
