package org.firstinspires.ftc.team8745;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by houserm on 3/12/17.
 */
@TeleOp(name="OpenHouse")

public class OpenHouse extends OpMode {

    DcMotor left_b;
    DcMotor right_b;
    DcMotor ballpickup;
    DcMotor ballshooter;
    TouchSensor uppersensor;
    TouchSensor lowersensor;
    public void  init(){

        left_b = hardwareMap.dcMotor.get("motor-leftBack");
        right_b = hardwareMap.dcMotor.get("motor-rightBack");
        ballpickup = hardwareMap.dcMotor.get("motor-ballpickup");
        ballshooter = hardwareMap.dcMotor.get("motor-ballshooter");
        uppersensor = hardwareMap.touchSensor.get("upper");
        lowersensor = hardwareMap.touchSensor.get("lower");

        right_b.setDirection(DcMotorSimple.Direction.REVERSE);
        ballshooter.setDirection(DcMotorSimple.Direction.REVERSE);
        while(uppersensor.isPressed()) {
            ballpickup.setPower(-.25);
        }
        ballpickup.setPower(0);
        telemetry.addData("uppersensor",uppersensor.isPressed());
        telemetry.addData("loworsensor",lowersensor.isPressed());
        telemetry.update();
    }
    @Override
    public void loop() {
        float leftDC = gamepad1.left_stick_y;
        float rightDC = gamepad1.right_stick_y;
        float pickupDC = gamepad1.left_trigger;
        float shooterDC = gamepad1.right_trigger;

        if(pickupDC == 0){
            pickupDC = -.25f;
        }
        if (!uppersensor.isPressed()){
            pickupDC = Range.clip(pickupDC,0,.25f);
        }
        if (!lowersensor.isPressed()) {
            pickupDC = Range.clip(pickupDC, -1,0);
        }
        telemetry.addData("pickupDC",pickupDC);
        left_b.setPower(leftDC);
        right_b.setPower(rightDC);
        ballpickup.setPower(pickupDC);
        ballshooter.setPower(shooterDC);
        telemetry.addData("uppersensor",uppersensor.isPressed());
        telemetry.addData("loworsensor",lowersensor.isPressed());
        telemetry.update();

    }
}

