package org.firstinspires.ftc.teamcode.first.math.geometry.struct;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import org.firstinspires.ftc.teamcode.first.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.first.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.first.math.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class Pose2dStruct  implements Struct<Pose2d> {
    @Override
    public Class<Pose2d> getTypeClass() {
        return Pose2d.class;
    }

    @Override
    public String getTypeString() {
        return "struct:Pose2d";
    }

    @Override
    public int getSize() {
        return Translation2d.struct.getSize() + Rotation2d.struct.getSize();
    }

    @Override
    public String getSchema() {
        return "Translation2d translation;Rotation2d rotation";
    }

    @Override
    public Struct<?>[] getNested() {
        return new Struct<?>[] {Translation2d.struct, Rotation2d.struct};
    }

    @Override
    public Pose2d unpack(ByteBuffer bb) {
        Translation2d translation = Translation2d.struct.unpack(bb);
        Rotation2d rotation = Rotation2d.struct.unpack(bb);
        return new Pose2d(translation, rotation);
    }

    @Override
    public void pack(ByteBuffer bb, Pose2d value) {
        Translation2d.struct.pack(bb, value.getTranslation());
        Rotation2d.struct.pack(bb, value.getRotation());
    }
}
