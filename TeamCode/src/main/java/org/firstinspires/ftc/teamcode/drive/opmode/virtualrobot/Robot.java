package org.firstinspires.ftc.teamcode.drive.opmode.virtualrobot;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Navigation nav = new Navigation();

    void init(HardwareMap hwMap) {
        nav.init(hwMap);
    }
}
