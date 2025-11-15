package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MyTelem {
    public static MultipleTelemetry tel;
    public static void init(Telemetry tm){
        tel = new MultipleTelemetry(tm, FtcDashboard.getInstance().getTelemetry());
    }

    public static void addData(String key, Object value){
        tel.addData(key, value);
    }
    public static void addLine(String key) {
        tel.addLine(key);
    }
    public static void addLine() {
        addLine("");
    }
    public static void update(){
        tel.update();
    }
}
