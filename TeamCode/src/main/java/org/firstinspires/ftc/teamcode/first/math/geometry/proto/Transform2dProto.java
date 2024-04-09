package org.firstinspires.ftc.teamcode.first.math.geometry.proto;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import org.firstinspires.ftc.teamcode.first.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.first.math.geometry.Transform2d;
import org.firstinspires.ftc.teamcode.first.math.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.first.math.proto.Geometry2D.ProtobufTransform2d;
import org.firstinspires.ftc.teamcode.first.util.protobuf.Protobuf;
import us.hebi.quickbuf.Descriptors.Descriptor;

public class Transform2dProto implements Protobuf<Transform2d, ProtobufTransform2d> {
    @Override
    public Class<Transform2d> getTypeClass() {
        return Transform2d.class;
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufTransform2d.getDescriptor();
    }

    @Override
    public Protobuf<?, ?>[] getNested() {
        return new Protobuf<?, ?>[] {Translation2d.proto, Rotation2d.proto};
    }

    @Override
    public ProtobufTransform2d createMessage() {
        return ProtobufTransform2d.newInstance();
    }

    @Override
    public Transform2d unpack(ProtobufTransform2d msg) {
        return new Transform2d(
                Translation2d.proto.unpack(msg.getTranslation()),
                Rotation2d.proto.unpack(msg.getRotation()));
    }

    @Override
    public void pack(ProtobufTransform2d msg, Transform2d value) {
        Translation2d.proto.pack(msg.getMutableTranslation(), value.getTranslation());
        Rotation2d.proto.pack(msg.getMutableRotation(), value.getRotation());
    }
}