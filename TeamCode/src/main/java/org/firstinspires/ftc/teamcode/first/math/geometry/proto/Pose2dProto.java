package org.firstinspires.ftc.teamcode.first.math.geometry.proto;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import org.firstinspires.ftc.teamcode.first.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.first.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.first.math.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.first.math.proto.Geometry2D.ProtobufPose2d;
import org.firstinspires.ftc.teamcode.first.util.protobuf.Protobuf;
import us.hebi.quickbuf.Descriptors.Descriptor;

public class Pose2dProto implements Protobuf<Pose2d, ProtobufPose2d> {
    @Override
    public Class<Pose2d> getTypeClass() {
        return Pose2d.class;
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufPose2d.getDescriptor();
    }

    @Override
    public Protobuf<?, ?>[] getNested() {
        return new Protobuf<?, ?>[] {Translation2d.proto, Rotation2d.proto};
    }

    @Override
    public ProtobufPose2d createMessage() {
        return ProtobufPose2d.newInstance();
    }

    @Override
    public Pose2d unpack(ProtobufPose2d msg) {
        return new Pose2d(
                Translation2d.proto.unpack(msg.getTranslation()),
                Rotation2d.proto.unpack(msg.getRotation()));
    }

    @Override
    public void pack(ProtobufPose2d msg, Pose2d value) {
        Translation2d.proto.pack(msg.getMutableTranslation(), value.getTranslation());
        Rotation2d.proto.pack(msg.getMutableRotation(), value.getRotation());
    }
}
