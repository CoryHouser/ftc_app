package org.firstinspires.ftc.team8745;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by houserm on 1/8/17.
 */
@Autonomous (name = "ColorSensortests")
public class ColorSensortests extends LinearOpMode{
    Hardware8745 robot = new Hardware8745();

    int foo;

    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap, telemetry);

        waitForStart();
        while (super.opModeIsActive()){
            telemetry.addData("Color_Case",robot.Color_Case());
            telemetry.addData("CSensorL Red", robot.CSensorL.red());
            telemetry.addData("CSensorL Blue",robot.CSensorL.blue());
            telemetry.addData("CSensorR Red",robot.CSensorR.red());
            telemetry.addData("CSensorR Blue",robot.CSensorR.blue());
            telemetry.update();



        }


    }


}

