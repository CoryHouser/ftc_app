package org.firstinspires.ftc.team8745;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by houserm on 3/12/17.
 */
@TeleOp(name="OpenHouse")

public class OpenHouse extends OpMode {

    DcMotor left_b;
    DcMotor right_b;

    public void  init(){

        left_b = hardwareMap.dcMotor.get("motor-leftBack");
        right_b = hardwareMap.dcMotor.get("motor-rightBack");

        right_b.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop() {
        float leftDC = gamepad1.left_stick_y;
        float rightDC = gamepad1.right_stick_y;

        left_b.setPower(leftDC);
        right_b.setPower(rightDC);



    }
}

