package org.firstinspires.ftc.teamcode.first.math.geometry.struct;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import org.firstinspires.ftc.teamcode.first.math.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class Translation2dStruct  implements Struct<Translation2d> {
    @Override
    public Class<Translation2d> getTypeClass() {
        return Translation2d.class;
    }

    @Override
    public String getTypeString() {
        return "struct:Translation2d";
    }

    @Override
    public int getSize() {
        return kSizeDouble * 2;
    }

    @Override
    public String getSchema() {
        return "double x;double y";
    }

    @Override
    public Translation2d unpack(ByteBuffer bb) {
        double x = bb.getDouble();
        double y = bb.getDouble();
        return new Translation2d(x, y);
    }

    @Override
    public void pack(ByteBuffer bb, Translation2d value) {
        bb.putDouble(value.getX());
        bb.putDouble(value.getY());
    }
}
