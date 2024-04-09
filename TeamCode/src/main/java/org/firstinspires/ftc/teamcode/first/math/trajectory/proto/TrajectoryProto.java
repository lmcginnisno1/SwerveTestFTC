package org.firstinspires.ftc.teamcode.first.math.trajectory.proto;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import org.firstinspires.ftc.teamcode.first.math.proto.Trajectory.ProtobufTrajectory;
import org.firstinspires.ftc.teamcode.first.math.proto.Trajectory.ProtobufTrajectoryState;
import org.firstinspires.ftc.teamcode.first.math.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.first.util.protobuf.Protobuf;
import java.util.ArrayList;
import us.hebi.quickbuf.Descriptors.Descriptor;

public class TrajectoryProto implements Protobuf<Trajectory, ProtobufTrajectory> {
    @Override
    public Class<Trajectory> getTypeClass() {
        return Trajectory.class;
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufTrajectory.getDescriptor();
    }

    @Override
    public Protobuf<?, ?>[] getNested() {
        return new Protobuf<?, ?>[] {Trajectory.State.proto};
    }

    @Override
    public ProtobufTrajectory createMessage() {
        return ProtobufTrajectory.newInstance();
    }

    @Override
    public Trajectory unpack(ProtobufTrajectory msg) {
        ArrayList<Trajectory.State> states = new ArrayList<>(msg.getStates().length());
        for (ProtobufTrajectoryState protoState : msg.getStates()) {
            states.add(Trajectory.State.proto.unpack(protoState));
        }

        return new Trajectory(states);
    }

    @Override
    public void pack(ProtobufTrajectory msg, Trajectory value) {
        var states = msg.getMutableStates().reserve(value.getStates().size());
        for (var item : value.getStates()) {
            Trajectory.State.proto.pack(states.next(), item);
        }
    }
}
