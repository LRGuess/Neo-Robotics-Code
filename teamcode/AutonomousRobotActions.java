package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class AutonomousRobotActions {
    public void moveForward(int duration, DcMotor _greenMotor, DcMotor _blackMotor) {
        _greenMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _blackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _greenMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        _blackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        _greenMotor.setTargetPosition(duration);
        _blackMotor.setTargetPosition(duration);

        _greenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _blackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveBackwards(int duration, DcMotor _greenMotor, DcMotor _blackMotor){
        _greenMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _blackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _greenMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        _blackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        _greenMotor.setTargetPosition(duration);
        _blackMotor.setTargetPosition(duration);

        _greenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _blackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void turnLeft(int duration, DcMotor _greenMotor, DcMotor _blackMotor){
        _greenMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _blackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _greenMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        _blackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        _greenMotor.setTargetPosition(duration);
        _blackMotor.setTargetPosition(duration);

        _greenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _blackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void turnRight(int duration, DcMotor _greenMotor, DcMotor _blackMotor){
        _greenMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _blackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _greenMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        _blackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        _greenMotor.setTargetPosition(duration);
        _blackMotor.setTargetPosition(duration);

        _greenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _blackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void armUp(double power, int duration, DcMotor _armMotor){
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

    public void closeHand(Servo _rightHandServo, Servo _leftHandServo){

    }

    public void openHand(Servo _rightHandServo, Servo _leftHandServo){

    }

    public void storeHand(@NonNull Servo _rightHandServo, @NonNull Servo _leftHandServo){
        _leftHandServo.setPosition(0);
//        if (_leftHandServo.getPosition() == 0)
//            _rightHandServo.setPosition(0);
    }

    public void releaseHand(@NonNull Servo _rightHandServo,@NonNull Servo _leftHandServo) {
        _rightHandServo.setPosition(0.5);
//        if (_rightHandServo.getPosition() == 0.5)
//            _leftHandServo.setPosition(0.5);
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
