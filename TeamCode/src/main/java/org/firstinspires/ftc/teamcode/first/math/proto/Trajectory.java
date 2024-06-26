package org.firstinspires.ftc.teamcode.first.math.proto;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Code generated by protocol buffer compiler. Do not edit!

import java.io.IOException;
import us.hebi.quickbuf.Descriptors;
import us.hebi.quickbuf.FieldName;
import us.hebi.quickbuf.InvalidProtocolBufferException;
import us.hebi.quickbuf.JsonSink;
import us.hebi.quickbuf.JsonSource;
import us.hebi.quickbuf.MessageFactory;
import us.hebi.quickbuf.ProtoMessage;
import us.hebi.quickbuf.ProtoSink;
import us.hebi.quickbuf.ProtoSource;
import us.hebi.quickbuf.ProtoUtil;
import us.hebi.quickbuf.RepeatedByte;
import us.hebi.quickbuf.RepeatedMessage;

public final class Trajectory {
    private static final RepeatedByte descriptorData = ProtoUtil.decodeBase64(809,
            "ChB0cmFqZWN0b3J5LnByb3RvEgl3cGkucHJvdG8aEGdlb21ldHJ5MmQucHJvdG8iugEKF1Byb3RvYnVm" +
                    "VHJhamVjdG9yeVN0YXRlEhIKBHRpbWUYASABKAFSBHRpbWUSGgoIdmVsb2NpdHkYAiABKAFSCHZlbG9j" +
                    "aXR5EiIKDGFjY2VsZXJhdGlvbhgDIAEoAVIMYWNjZWxlcmF0aW9uEi0KBHBvc2UYBCABKAsyGS53cGku" +
                    "cHJvdG8uUHJvdG9idWZQb3NlMmRSBHBvc2USHAoJY3VydmF0dXJlGAUgASgBUgljdXJ2YXR1cmUiUAoS" +
                    "UHJvdG9idWZUcmFqZWN0b3J5EjoKBnN0YXRlcxgCIAMoCzIiLndwaS5wcm90by5Qcm90b2J1ZlRyYWpl" +
                    "Y3RvcnlTdGF0ZVIGc3RhdGVzQhoKGGVkdS53cGkuZmlyc3QubWF0aC5wcm90b0rEAwoGEgQAABIBCggK" +
                    "AQwSAwAAEgoICgECEgMCABIKCQoCAwASAwQAGgoICgEIEgMGADEKCQoCCAESAwYAMQoKCgIEABIECAAO" +
                    "AQoKCgMEAAESAwgIHwoLCgQEAAIAEgMJAhIKDAoFBAACAAUSAwkCCAoMCgUEAAIAARIDCQkNCgwKBQQA" +
                    "AgADEgMJEBEKCwoEBAACARIDCgIWCgwKBQQAAgEFEgMKAggKDAoFBAACAQESAwoJEQoMCgUEAAIBAxID" +
                    "ChQVCgsKBAQAAgISAwsCGgoMCgUEAAICBRIDCwIICgwKBQQAAgIBEgMLCRUKDAoFBAACAgMSAwsYGQoL" +
                    "CgQEAAIDEgMMAhoKDAoFBAACAwYSAwwCEAoMCgUEAAIDARIDDBEVCgwKBQQAAgMDEgMMGBkKCwoEBAAC" +
                    "BBIDDQIXCgwKBQQAAgQFEgMNAggKDAoFBAACBAESAw0JEgoMCgUEAAIEAxIDDRUWCgoKAgQBEgQQABIB" +
                    "CgoKAwQBARIDEAgaCgsKBAQBAgASAxECLgoMCgUEAQIABBIDEQIKCgwKBQQBAgAGEgMRCyIKDAoFBAEC" +
                    "AAESAxEjKQoMCgUEAQIAAxIDESwtYgZwcm90bzM=");

    static final Descriptors.FileDescriptor descriptor = Descriptors.FileDescriptor.internalBuildGeneratedFileFrom("trajectory.proto", "wpi.proto", descriptorData, Geometry2D.getDescriptor());

    static final Descriptors.Descriptor wpi_proto_ProtobufTrajectoryState_descriptor = descriptor.internalContainedType(50, 186, "ProtobufTrajectoryState", "wpi.proto.ProtobufTrajectoryState");

    static final Descriptors.Descriptor wpi_proto_ProtobufTrajectory_descriptor = descriptor.internalContainedType(238, 80, "ProtobufTrajectory", "wpi.proto.ProtobufTrajectory");

    /**
     * @return this proto file's descriptor.
     */
    public static Descriptors.FileDescriptor getDescriptor() {
        return descriptor;
    }

    /**
     * Protobuf type {@code ProtobufTrajectoryState}
     */
    public static final class ProtobufTrajectoryState extends ProtoMessage<ProtobufTrajectoryState> implements Cloneable {
        private static final long serialVersionUID = 0L;

        /**
         * <code>optional double time = 1;</code>
         */
        private double time;

        /**
         * <code>optional double velocity = 2;</code>
         */
        private double velocity;

        /**
         * <code>optional double acceleration = 3;</code>
         */
        private double acceleration;

        /**
         * <code>optional double curvature = 5;</code>
         */
        private double curvature;

        /**
         * <code>optional .wpi.proto.ProtobufPose2d pose = 4;</code>
         */
        private final Geometry2D.ProtobufPose2d pose = Geometry2D.ProtobufPose2d.newInstance();

        private ProtobufTrajectoryState() {
        }

        /**
         * @return a new empty instance of {@code ProtobufTrajectoryState}
         */
        public static ProtobufTrajectoryState newInstance() {
            return new ProtobufTrajectoryState();
        }

        /**
         * <code>optional double time = 1;</code>
         * @return whether the time field is set
         */
        public boolean hasTime() {
            return (bitField0_ & 0x00000001) != 0;
        }

        /**
         * <code>optional double time = 1;</code>
         * @return this
         */
        public ProtobufTrajectoryState clearTime() {
            bitField0_ &= ~0x00000001;
            time = 0D;
            return this;
        }

        /**
         * <code>optional double time = 1;</code>
         * @return the time
         */
        public double getTime() {
            return time;
        }

        /**
         * <code>optional double time = 1;</code>
         * @param value the time to set
         * @return this
         */
        public ProtobufTrajectoryState setTime(final double value) {
            bitField0_ |= 0x00000001;
            time = value;
            return this;
        }

        /**
         * <code>optional double velocity = 2;</code>
         * @return whether the velocity field is set
         */
        public boolean hasVelocity() {
            return (bitField0_ & 0x00000002) != 0;
        }

        /**
         * <code>optional double velocity = 2;</code>
         * @return this
         */
        public ProtobufTrajectoryState clearVelocity() {
            bitField0_ &= ~0x00000002;
            velocity = 0D;
            return this;
        }

        /**
         * <code>optional double velocity = 2;</code>
         * @return the velocity
         */
        public double getVelocity() {
            return velocity;
        }

        /**
         * <code>optional double velocity = 2;</code>
         * @param value the velocity to set
         * @return this
         */
        public ProtobufTrajectoryState setVelocity(final double value) {
            bitField0_ |= 0x00000002;
            velocity = value;
            return this;
        }

        /**
         * <code>optional double acceleration = 3;</code>
         * @return whether the acceleration field is set
         */
        public boolean hasAcceleration() {
            return (bitField0_ & 0x00000004) != 0;
        }

        /**
         * <code>optional double acceleration = 3;</code>
         * @return this
         */
        public ProtobufTrajectoryState clearAcceleration() {
            bitField0_ &= ~0x00000004;
            acceleration = 0D;
            return this;
        }

        /**
         * <code>optional double acceleration = 3;</code>
         * @return the acceleration
         */
        public double getAcceleration() {
            return acceleration;
        }

        /**
         * <code>optional double acceleration = 3;</code>
         * @param value the acceleration to set
         * @return this
         */
        public ProtobufTrajectoryState setAcceleration(final double value) {
            bitField0_ |= 0x00000004;
            acceleration = value;
            return this;
        }

        /**
         * <code>optional double curvature = 5;</code>
         * @return whether the curvature field is set
         */
        public boolean hasCurvature() {
            return (bitField0_ & 0x00000008) != 0;
        }

        /**
         * <code>optional double curvature = 5;</code>
         * @return this
         */
        public ProtobufTrajectoryState clearCurvature() {
            bitField0_ &= ~0x00000008;
            curvature = 0D;
            return this;
        }

        /**
         * <code>optional double curvature = 5;</code>
         * @return the curvature
         */
        public double getCurvature() {
            return curvature;
        }

        /**
         * <code>optional double curvature = 5;</code>
         * @param value the curvature to set
         * @return this
         */
        public ProtobufTrajectoryState setCurvature(final double value) {
            bitField0_ |= 0x00000008;
            curvature = value;
            return this;
        }

        /**
         * <code>optional .wpi.proto.ProtobufPose2d pose = 4;</code>
         * @return whether the pose field is set
         */
        public boolean hasPose() {
            return (bitField0_ & 0x00000010) != 0;
        }

        /**
         * <code>optional .wpi.proto.ProtobufPose2d pose = 4;</code>
         * @return this
         */
        public ProtobufTrajectoryState clearPose() {
            bitField0_ &= ~0x00000010;
            pose.clear();
            return this;
        }

        /**
         * <code>optional .wpi.proto.ProtobufPose2d pose = 4;</code>
         *
         * This method returns the internal storage object without modifying any has state.
         * The returned object should not be modified and be treated as read-only.
         *
         * Use {@link #getMutablePose()} if you want to modify it.
         *
         * @return internal storage object for reading
         */
        public Geometry2D.ProtobufPose2d getPose() {
            return pose;
        }

        /**
         * <code>optional .wpi.proto.ProtobufPose2d pose = 4;</code>
         *
         * This method returns the internal storage object and sets the corresponding
         * has state. The returned object will become part of this message and its
         * contents may be modified as long as the has state is not cleared.
         *
         * @return internal storage object for modifications
         */
        public Geometry2D.ProtobufPose2d getMutablePose() {
            bitField0_ |= 0x00000010;
            return pose;
        }

        /**
         * <code>optional .wpi.proto.ProtobufPose2d pose = 4;</code>
         * @param value the pose to set
         * @return this
         */
        public ProtobufTrajectoryState setPose(final Geometry2D.ProtobufPose2d value) {
            bitField0_ |= 0x00000010;
            pose.copyFrom(value);
            return this;
        }

        @Override
        public ProtobufTrajectoryState copyFrom(final ProtobufTrajectoryState other) {
            cachedSize = other.cachedSize;
            if ((bitField0_ | other.bitField0_) != 0) {
                bitField0_ = other.bitField0_;
                time = other.time;
                velocity = other.velocity;
                acceleration = other.acceleration;
                curvature = other.curvature;
                pose.copyFrom(other.pose);
            }
            return this;
        }

        @Override
        public ProtobufTrajectoryState mergeFrom(final ProtobufTrajectoryState other) {
            if (other.isEmpty()) {
                return this;
            }
            cachedSize = -1;
            if (other.hasTime()) {
                setTime(other.time);
            }
            if (other.hasVelocity()) {
                setVelocity(other.velocity);
            }
            if (other.hasAcceleration()) {
                setAcceleration(other.acceleration);
            }
            if (other.hasCurvature()) {
                setCurvature(other.curvature);
            }
            if (other.hasPose()) {
                getMutablePose().mergeFrom(other.pose);
            }
            return this;
        }

        @Override
        public ProtobufTrajectoryState clear() {
            if (isEmpty()) {
                return this;
            }
            cachedSize = -1;
            bitField0_ = 0;
            time = 0D;
            velocity = 0D;
            acceleration = 0D;
            curvature = 0D;
            pose.clear();
            return this;
        }

        @Override
        public ProtobufTrajectoryState clearQuick() {
            if (isEmpty()) {
                return this;
            }
            cachedSize = -1;
            bitField0_ = 0;
            pose.clearQuick();
            return this;
        }

        @Override
        public boolean equals(Object o) {
            if (o == this) {
                return true;
            }
            if (!(o instanceof ProtobufTrajectoryState)) {
                return false;
            }
            ProtobufTrajectoryState other = (ProtobufTrajectoryState) o;
            return bitField0_ == other.bitField0_
                    && (!hasTime() || ProtoUtil.isEqual(time, other.time))
                    && (!hasVelocity() || ProtoUtil.isEqual(velocity, other.velocity))
                    && (!hasAcceleration() || ProtoUtil.isEqual(acceleration, other.acceleration))
                    && (!hasCurvature() || ProtoUtil.isEqual(curvature, other.curvature))
                    && (!hasPose() || pose.equals(other.pose));
        }

        @Override
        public void writeTo(final ProtoSink output) throws IOException {
            if ((bitField0_ & 0x00000001) != 0) {
                output.writeRawByte((byte) 9);
                output.writeDoubleNoTag(time);
            }
            if ((bitField0_ & 0x00000002) != 0) {
                output.writeRawByte((byte) 17);
                output.writeDoubleNoTag(velocity);
            }
            if ((bitField0_ & 0x00000004) != 0) {
                output.writeRawByte((byte) 25);
                output.writeDoubleNoTag(acceleration);
            }
            if ((bitField0_ & 0x00000008) != 0) {
                output.writeRawByte((byte) 41);
                output.writeDoubleNoTag(curvature);
            }
            if ((bitField0_ & 0x00000010) != 0) {
                output.writeRawByte((byte) 34);
                output.writeMessageNoTag(pose);
            }
        }

        @Override
        protected int computeSerializedSize() {
            int size = 0;
            if ((bitField0_ & 0x00000001) != 0) {
                size += 9;
            }
            if ((bitField0_ & 0x00000002) != 0) {
                size += 9;
            }
            if ((bitField0_ & 0x00000004) != 0) {
                size += 9;
            }
            if ((bitField0_ & 0x00000008) != 0) {
                size += 9;
            }
            if ((bitField0_ & 0x00000010) != 0) {
                size += 1 + ProtoSink.computeMessageSizeNoTag(pose);
            }
            return size;
        }

        @Override
        @SuppressWarnings("fallthrough")
        public ProtobufTrajectoryState mergeFrom(final ProtoSource input) throws IOException {
            // Enabled Fall-Through Optimization (QuickBuffers)
            int tag = input.readTag();
            while (true) {
                switch (tag) {
                    case 9: {
                        // time
                        time = input.readDouble();
                        bitField0_ |= 0x00000001;
                        tag = input.readTag();
                        if (tag != 17) {
                            break;
                        }
                    }
                    case 17: {
                        // velocity
                        velocity = input.readDouble();
                        bitField0_ |= 0x00000002;
                        tag = input.readTag();
                        if (tag != 25) {
                            break;
                        }
                    }
                    case 25: {
                        // acceleration
                        acceleration = input.readDouble();
                        bitField0_ |= 0x00000004;
                        tag = input.readTag();
                        if (tag != 41) {
                            break;
                        }
                    }
                    case 41: {
                        // curvature
                        curvature = input.readDouble();
                        bitField0_ |= 0x00000008;
                        tag = input.readTag();
                        if (tag != 34) {
                            break;
                        }
                    }
                    case 34: {
                        // pose
                        input.readMessage(pose);
                        bitField0_ |= 0x00000010;
                        tag = input.readTag();
                        if (tag != 0) {
                            break;
                        }
                    }
                    case 0: {
                        return this;
                    }
                    default: {
                        if (!input.skipField(tag)) {
                            return this;
                        }
                        tag = input.readTag();
                        break;
                    }
                }
            }
        }

        @Override
        public void writeTo(final JsonSink output) throws IOException {
            output.beginObject();
            if ((bitField0_ & 0x00000001) != 0) {
                output.writeDouble(FieldNames.time, time);
            }
            if ((bitField0_ & 0x00000002) != 0) {
                output.writeDouble(FieldNames.velocity, velocity);
            }
            if ((bitField0_ & 0x00000004) != 0) {
                output.writeDouble(FieldNames.acceleration, acceleration);
            }
            if ((bitField0_ & 0x00000008) != 0) {
                output.writeDouble(FieldNames.curvature, curvature);
            }
            if ((bitField0_ & 0x00000010) != 0) {
                output.writeMessage(FieldNames.pose, pose);
            }
            output.endObject();
        }

        @Override
        public ProtobufTrajectoryState mergeFrom(final JsonSource input) throws IOException {
            if (!input.beginObject()) {
                return this;
            }
            while (!input.isAtEnd()) {
                switch (input.readFieldHash()) {
                    case 3560141: {
                        if (input.isAtField(FieldNames.time)) {
                            if (!input.trySkipNullValue()) {
                                time = input.readDouble();
                                bitField0_ |= 0x00000001;
                            }
                        } else {
                            input.skipUnknownField();
                        }
                        break;
                    }
                    case 2134260957: {
                        if (input.isAtField(FieldNames.velocity)) {
                            if (!input.trySkipNullValue()) {
                                velocity = input.readDouble();
                                bitField0_ |= 0x00000002;
                            }
                        } else {
                            input.skipUnknownField();
                        }
                        break;
                    }
                    case -267299712: {
                        if (input.isAtField(FieldNames.acceleration)) {
                            if (!input.trySkipNullValue()) {
                                acceleration = input.readDouble();
                                bitField0_ |= 0x00000004;
                            }
                        } else {
                            input.skipUnknownField();
                        }
                        break;
                    }
                    case 768611295: {
                        if (input.isAtField(FieldNames.curvature)) {
                            if (!input.trySkipNullValue()) {
                                curvature = input.readDouble();
                                bitField0_ |= 0x00000008;
                            }
                        } else {
                            input.skipUnknownField();
                        }
                        break;
                    }
                    case 3446929: {
                        if (input.isAtField(FieldNames.pose)) {
                            if (!input.trySkipNullValue()) {
                                input.readMessage(pose);
                                bitField0_ |= 0x00000010;
                            }
                        } else {
                            input.skipUnknownField();
                        }
                        break;
                    }
                    default: {
                        input.skipUnknownField();
                        break;
                    }
                }
            }
            input.endObject();
            return this;
        }

        @Override
        public ProtobufTrajectoryState clone() {
            return new ProtobufTrajectoryState().copyFrom(this);
        }

        @Override
        public boolean isEmpty() {
            return ((bitField0_) == 0);
        }

        public static ProtobufTrajectoryState parseFrom(final byte[] data) throws
                InvalidProtocolBufferException {
            return ProtoMessage.mergeFrom(new ProtobufTrajectoryState(), data).checkInitialized();
        }

        public static ProtobufTrajectoryState parseFrom(final ProtoSource input) throws IOException {
            return ProtoMessage.mergeFrom(new ProtobufTrajectoryState(), input).checkInitialized();
        }

        public static ProtobufTrajectoryState parseFrom(final JsonSource input) throws IOException {
            return ProtoMessage.mergeFrom(new ProtobufTrajectoryState(), input).checkInitialized();
        }

        /**
         * @return factory for creating ProtobufTrajectoryState messages
         */
        public static MessageFactory<ProtobufTrajectoryState> getFactory() {
            return ProtobufTrajectoryStateFactory.INSTANCE;
        }

        /**
         * @return this type's descriptor.
         */
        public static Descriptors.Descriptor getDescriptor() {
            return Trajectory.wpi_proto_ProtobufTrajectoryState_descriptor;
        }

        private enum ProtobufTrajectoryStateFactory implements MessageFactory<ProtobufTrajectoryState> {
            INSTANCE;

            @Override
            public ProtobufTrajectoryState create() {
                return ProtobufTrajectoryState.newInstance();
            }
        }

        /**
         * Contains name constants used for serializing JSON
         */
        static class FieldNames {
            static final FieldName time = FieldName.forField("time");

            static final FieldName velocity = FieldName.forField("velocity");

            static final FieldName acceleration = FieldName.forField("acceleration");

            static final FieldName curvature = FieldName.forField("curvature");

            static final FieldName pose = FieldName.forField("pose");
        }
    }

    /**
     * Protobuf type {@code ProtobufTrajectory}
     */
    public static final class ProtobufTrajectory extends ProtoMessage<ProtobufTrajectory> implements Cloneable {
        private static final long serialVersionUID = 0L;

        /**
         * <code>repeated .wpi.proto.ProtobufTrajectoryState states = 2;</code>
         */
        private final RepeatedMessage<ProtobufTrajectoryState> states = RepeatedMessage.newEmptyInstance(ProtobufTrajectoryState.getFactory());

        private ProtobufTrajectory() {
        }

        /**
         * @return a new empty instance of {@code ProtobufTrajectory}
         */
        public static ProtobufTrajectory newInstance() {
            return new ProtobufTrajectory();
        }

        /**
         * <code>repeated .wpi.proto.ProtobufTrajectoryState states = 2;</code>
         * @return whether the states field is set
         */
        public boolean hasStates() {
            return (bitField0_ & 0x00000001) != 0;
        }

        /**
         * <code>repeated .wpi.proto.ProtobufTrajectoryState states = 2;</code>
         * @return this
         */
        public ProtobufTrajectory clearStates() {
            bitField0_ &= ~0x00000001;
            states.clear();
            return this;
        }

        /**
         * <code>repeated .wpi.proto.ProtobufTrajectoryState states = 2;</code>
         *
         * This method returns the internal storage object without modifying any has state.
         * The returned object should not be modified and be treated as read-only.
         *
         * Use {@link #getMutableStates()} if you want to modify it.
         *
         * @return internal storage object for reading
         */
        public RepeatedMessage<ProtobufTrajectoryState> getStates() {
            return states;
        }

        /**
         * <code>repeated .wpi.proto.ProtobufTrajectoryState states = 2;</code>
         *
         * This method returns the internal storage object and sets the corresponding
         * has state. The returned object will become part of this message and its
         * contents may be modified as long as the has state is not cleared.
         *
         * @return internal storage object for modifications
         */
        public RepeatedMessage<ProtobufTrajectoryState> getMutableStates() {
            bitField0_ |= 0x00000001;
            return states;
        }

        /**
         * <code>repeated .wpi.proto.ProtobufTrajectoryState states = 2;</code>
         * @param value the states to add
         * @return this
         */
        public ProtobufTrajectory addStates(final ProtobufTrajectoryState value) {
            bitField0_ |= 0x00000001;
            states.add(value);
            return this;
        }

        /**
         * <code>repeated .wpi.proto.ProtobufTrajectoryState states = 2;</code>
         * @param values the states to add
         * @return this
         */
        public ProtobufTrajectory addAllStates(final ProtobufTrajectoryState... values) {
            bitField0_ |= 0x00000001;
            states.addAll(values);
            return this;
        }

        @Override
        public ProtobufTrajectory copyFrom(final ProtobufTrajectory other) {
            cachedSize = other.cachedSize;
            if ((bitField0_ | other.bitField0_) != 0) {
                bitField0_ = other.bitField0_;
                states.copyFrom(other.states);
            }
            return this;
        }

        @Override
        public ProtobufTrajectory mergeFrom(final ProtobufTrajectory other) {
            if (other.isEmpty()) {
                return this;
            }
            cachedSize = -1;
            if (other.hasStates()) {
                getMutableStates().addAll(other.states);
            }
            return this;
        }

        @Override
        public ProtobufTrajectory clear() {
            if (isEmpty()) {
                return this;
            }
            cachedSize = -1;
            bitField0_ = 0;
            states.clear();
            return this;
        }

        @Override
        public ProtobufTrajectory clearQuick() {
            if (isEmpty()) {
                return this;
            }
            cachedSize = -1;
            bitField0_ = 0;
            states.clearQuick();
            return this;
        }

        @Override
        public boolean equals(Object o) {
            if (o == this) {
                return true;
            }
            if (!(o instanceof ProtobufTrajectory)) {
                return false;
            }
            ProtobufTrajectory other = (ProtobufTrajectory) o;
            return bitField0_ == other.bitField0_
                    && (!hasStates() || states.equals(other.states));
        }

        @Override
        public void writeTo(final ProtoSink output) throws IOException {
            if ((bitField0_ & 0x00000001) != 0) {
                for (int i = 0; i < states.length(); i++) {
                    output.writeRawByte((byte) 18);
                    output.writeMessageNoTag(states.get(i));
                }
            }
        }

        @Override
        protected int computeSerializedSize() {
            int size = 0;
            if ((bitField0_ & 0x00000001) != 0) {
                size += (1 * states.length()) + ProtoSink.computeRepeatedMessageSizeNoTag(states);
            }
            return size;
        }

        @Override
        @SuppressWarnings("fallthrough")
        public ProtobufTrajectory mergeFrom(final ProtoSource input) throws IOException {
            // Enabled Fall-Through Optimization (QuickBuffers)
            int tag = input.readTag();
            while (true) {
                switch (tag) {
                    case 18: {
                        // states
                        tag = input.readRepeatedMessage(states, tag);
                        bitField0_ |= 0x00000001;
                        if (tag != 0) {
                            break;
                        }
                    }
                    case 0: {
                        return this;
                    }
                    default: {
                        if (!input.skipField(tag)) {
                            return this;
                        }
                        tag = input.readTag();
                        break;
                    }
                }
            }
        }

        @Override
        public void writeTo(final JsonSink output) throws IOException {
            output.beginObject();
            if ((bitField0_ & 0x00000001) != 0) {
                output.writeRepeatedMessage(FieldNames.states, states);
            }
            output.endObject();
        }

        @Override
        public ProtobufTrajectory mergeFrom(final JsonSource input) throws IOException {
            if (!input.beginObject()) {
                return this;
            }
            while (!input.isAtEnd()) {
                switch (input.readFieldHash()) {
                    case -892482046: {
                        if (input.isAtField(FieldNames.states)) {
                            if (!input.trySkipNullValue()) {
                                input.readRepeatedMessage(states);
                                bitField0_ |= 0x00000001;
                            }
                        } else {
                            input.skipUnknownField();
                        }
                        break;
                    }
                    default: {
                        input.skipUnknownField();
                        break;
                    }
                }
            }
            input.endObject();
            return this;
        }

        @Override
        public ProtobufTrajectory clone() {
            return new ProtobufTrajectory().copyFrom(this);
        }

        @Override
        public boolean isEmpty() {
            return ((bitField0_) == 0);
        }

        public static ProtobufTrajectory parseFrom(final byte[] data) throws
                InvalidProtocolBufferException {
            return ProtoMessage.mergeFrom(new ProtobufTrajectory(), data).checkInitialized();
        }

        public static ProtobufTrajectory parseFrom(final ProtoSource input) throws IOException {
            return ProtoMessage.mergeFrom(new ProtobufTrajectory(), input).checkInitialized();
        }

        public static ProtobufTrajectory parseFrom(final JsonSource input) throws IOException {
            return ProtoMessage.mergeFrom(new ProtobufTrajectory(), input).checkInitialized();
        }

        /**
         * @return factory for creating ProtobufTrajectory messages
         */
        public static MessageFactory<ProtobufTrajectory> getFactory() {
            return ProtobufTrajectoryFactory.INSTANCE;
        }

        /**
         * @return this type's descriptor.
         */
        public static Descriptors.Descriptor getDescriptor() {
            return Trajectory.wpi_proto_ProtobufTrajectory_descriptor;
        }

        private enum ProtobufTrajectoryFactory implements MessageFactory<ProtobufTrajectory> {
            INSTANCE;

            @Override
            public ProtobufTrajectory create() {
                return ProtobufTrajectory.newInstance();
            }
        }

        /**
         * Contains name constants used for serializing JSON
         */
        static class FieldNames {
            static final FieldName states = FieldName.forField("states");
        }
    }
}
