using System.Collections.Generic;
using AnimationTools;
using UnityEngine;
using Unity.Mathematics;
using System.IO;
using System.Text;
using System;
using System.Runtime.InteropServices;
using NUnit.Framework;

namespace AnimationTools
{
using static AnimationTools.BinarySerializerExtensions;

public class PoseSerializer
{
    /// <summary>
    /// Stores the full pose representation of all poses for Motion Matching in a binary format
    /// in the specified path with name filename and extension .mmpose
    /// It also stores the skeleton used in poseSet with extension .mmskeleton
    /// </summary>
    public void Serialize(PoseSet poseSet, string path, string fileName)
    {
        Directory.CreateDirectory(path); // create directory and parent directories if they don't exist

        // Write Skeleton
        using (var stream = File.Open(Path.Combine(path, fileName + ".mmskeleton"), FileMode.Create))
        {
            using (var writer = new BinaryWriter(stream, Encoding.UTF8))
            {
                var skeleton = poseSet.SkeletonAsset;
                // Write Number Joints
                writer.Write((uint)skeleton.BoneCount);
                // Write Joints
                for (var i = 0; i < skeleton.BoneCount; ++i)
                {
                    var bone = skeleton.GetBone(i);
                    writer.Write(bone.name);
                    writer.Write((uint)i);
                    writer.Write((uint)bone.parentIndex); // root's -1 round-trips as 0xFFFFFFFF
                    WriteFloat3(writer, bone.restLocalPosition);
                    writer.Write((uint)bone.humanBone);
                }
            }
        }

        // Write Poses
        using (var stream = File.Open(Path.Combine(path, fileName + ".mmpose"), FileMode.Create))
        {
            using (var writer = new BinaryWriter(stream, Encoding.UTF8))
            {
                // Serialize Number Animation Clips
                writer.Write((uint)poseSet.NumberClips);
                // Serialize Animation Clips
                for (int i = 0; i < poseSet.NumberClips; ++i)
                {
                    PoseSet.AnimationClip clip = poseSet.GetAnimationClip(i);
                    writer.Write((uint)clip.Start);
                    writer.Write((uint)clip.End);
                    writer.Write(clip.FrameTime);
                }

                // Serialize Number Poses & Number Joints & Number Tags
                writer.Write((uint)poseSet.NumberPoses);
                writer.Write((uint)poseSet.SkeletonAsset.BoneCount);
                writer.Write((uint)poseSet.NumberTags);
                // Serialize Poses
                for (int i = 0; i < poseSet.NumberPoses; ++i)
                {
                    var frame = poseSet.GetPoseBuffer(i);
                    WriteFloat3Slice(writer, frame.Positions);
                    WriteQuaternionSlice(writer, frame.Rotations);
                    WriteFloat3Slice(writer, frame.Velocities);
                    WriteFloat3Slice(writer, frame.AngularVelocities);
                    writer.Write(frame.GetBool(poseSet.LeftFootContactHandle) ? 1u : 0u);
                    writer.Write(frame.GetBool(poseSet.RightFootContactHandle) ? 1u : 0u);
                }

                // Serialize Tags
                for (int i = 0; i < poseSet.NumberTags; ++i)
                {
                    PoseSet.AnimationTag animationTag = poseSet.GetTag(i);
                    writer.Write(animationTag.Name);
                    writer.Write((uint)animationTag.NumberRanges);
                    for (int r = 0; r < animationTag.NumberRanges; ++r)
                    {
                        animationTag.GetRange(r, out int startRange, out int endRange);
                        writer.Write((uint)startRange);
                        writer.Write((uint)endRange);
                    }
                }
            }
        }
    }

    /// <summary>
    /// Reads only the .mmskeleton file into an in-memory <see cref="SkeletonAsset"/>, without
    /// touching the far larger .mmpose alongside it. Returns false if the file is not there or
    /// holds no joints. The bone list mirrors the file's joint order (index i, id i + 1); joint
    /// rest rotations are identity since the format stores offsets only.
    /// </summary>
    /// <remarks>
    /// Split out of <see cref="Deserialize"/> for callers that want the joint hierarchy on its own
    /// -- an inspector listing bone names, for instance, which would otherwise pay for megabytes of
    /// pose data on every repaint.
    /// </remarks>
    public static bool TryDeserializeSkeleton(string path, string fileName, out SkeletonAsset skeleton)
    {
        skeleton = null;

        string skeletonPath = Path.Combine(path, fileName + ".mmskeleton");
        if (!File.Exists(skeletonPath))
            return false;

        byte[] skeletonData = File.ReadAllBytes(skeletonPath);
        using (var ms = new MemoryStream(skeletonData))
        {
            using (var reader = new BinaryReader(ms, Encoding.UTF8))
            {
                uint nJoints = reader.ReadUInt32();
                if (nJoints == 0)
                    return false;

                var bones = new List<Bone>((int)nJoints);
                for (int i = 0; i < nJoints; i++)
                {
                    string jointName = reader.ReadString();
                    reader.ReadUInt32(); // joint index; always equals the list position
                    uint jointParentIndex = reader.ReadUInt32(); // root's 0xFFFFFFFF -> -1
                    float3 jointLocalOffset = ReadFloat3(reader);
                    HumanBodyBones jointType = (HumanBodyBones)reader.ReadUInt32();
                    bones.Add(new Bone
                    {
                        id = i + 1,
                        name = jointName,
                        parentIndex = (int)jointParentIndex,
                        restLocalPosition = jointLocalOffset,
                        restLocalRotation = quaternion.identity,
                        humanBone = jointType
                    });
                }

                skeleton = PoseSet.CreateRuntimeSkeletonAsset(bones, fileName + "_Skeleton");
            }
        }

        return true;
    }

    /// <summary>
    /// Reads the full pose representation of all poses for Motion Matching from a binary format
    /// in the specified path with name filename and extension .mmpose and .mmskeleton
    /// Returns true if poseSet was successfully deserialized, false otherwise
    /// </summary>
    public bool Deserialize(string path, string fileName, out PoseSet poseSet)
    {
        poseSet = new PoseSet();

        // --------------------
        // Read Skeleton File
        // --------------------
        if (!TryDeserializeSkeleton(path, fileName, out SkeletonAsset skeleton))
            return false;

        poseSet.SetSkeleton(skeleton);

        // --------------------
        // Read Pose File
        // --------------------
        string posePath = Path.Combine(path, fileName + ".mmpose");
        if (!File.Exists(posePath))
            return false;

        byte[] poseData = File.ReadAllBytes(posePath);
        using (var ms = new MemoryStream(poseData))
        {
            using (var reader = new BinaryReader(ms, Encoding.UTF8))
            {
                uint nClips = reader.ReadUInt32();
                poseSet.SetClipCapacity(nClips);
                for (int i = 0; i < nClips; i++)
                {
                    uint start = reader.ReadUInt32();
                    uint end = reader.ReadUInt32();
                    float frameTime = reader.ReadSingle();
                    poseSet.AddAnimationClipDeserialized(new PoseSet.AnimationClip((int)start, (int)end, frameTime));
                }

                uint nPoses = reader.ReadUInt32();
                uint nJoints = reader.ReadUInt32();
                uint nTags = reader.ReadUInt32();
                Debug.Assert(nJoints == skeleton.BoneCount, "Number of joints in skeleton and pose do not match");

                // Precompute sizes for the buffers (they remain constant across iterations)
                int float3BufferSize = (int)nJoints * 3 * sizeof(float);
                int quaternionBufferSize = (int)nJoints * 4 * sizeof(float);

                // Allocate reusable buffers once outside the loop
                byte[] float3Buffer = new byte[float3BufferSize];
                byte[] quaternionBuffer = new byte[quaternionBufferSize];

                poseSet.SetPoseCapacity(nPoses);
                var frames = poseSet.AppendRawFrames((int)nPoses);
                for (int i = 0; i < nPoses; i++)
                {
                    var frame = frames[i];
                    var positions = frame.Positions;
                    var rotations = frame.Rotations;
                    var velocities = frame.Velocities;
                    var angularVelocities = frame.AngularVelocities;

                    // --- Read JointLocalPositions ---
                    var read = reader.Read(float3Buffer, 0, float3BufferSize);
                    Assert.IsTrue(read == float3BufferSize);
                    var positionsSpan = MemoryMarshal.Cast<byte, float>(float3Buffer);
                    for (int j = 0; j < nJoints; j++)
                    {
                        positions[j] = new float3(
                            positionsSpan[j * 3],
                            positionsSpan[j * 3 + 1],
                            positionsSpan[j * 3 + 2]
                        );
                    }

                    // --- Read JointLocalRotations ---
                    read = reader.Read(quaternionBuffer, 0, quaternionBufferSize);
                    Assert.IsTrue(read == quaternionBufferSize);
                    Span<float> rotationsSpan = MemoryMarshal.Cast<byte, float>(quaternionBuffer);
                    for (int j = 0; j < nJoints; j++)
                    {
                        rotations[j] = new quaternion(
                            rotationsSpan[j * 4],
                            rotationsSpan[j * 4 + 1],
                            rotationsSpan[j * 4 + 2],
                            rotationsSpan[j * 4 + 3]
                        );
                    }

                    // --- Read JointLocalVelocities ---
                    read = reader.Read(float3Buffer, 0, float3BufferSize);
                    Assert.IsTrue(read == float3BufferSize);
                    Span<float> velocitiesSpan = MemoryMarshal.Cast<byte, float>(float3Buffer);
                    for (int j = 0; j < nJoints; j++)
                    {
                        velocities[j] = new float3(
                            velocitiesSpan[j * 3],
                            velocitiesSpan[j * 3 + 1],
                            velocitiesSpan[j * 3 + 2]
                        );
                    }

                    // --- Read JointLocalAngularVelocities ---
                    read = reader.Read(float3Buffer, 0, float3BufferSize);
                    Assert.IsTrue(read == float3BufferSize);
                    Span<float> angularVelocitiesSpan = MemoryMarshal.Cast<byte, float>(float3Buffer);
                    for (int j = 0; j < nJoints; j++)
                    {
                        angularVelocities[j] = new float3(
                            angularVelocitiesSpan[j * 3],
                            angularVelocitiesSpan[j * 3 + 1],
                            angularVelocitiesSpan[j * 3 + 2]
                        );
                    }

                    // --- Read contact flags ---
                    frame.SetBool(poseSet.LeftFootContactHandle, reader.ReadUInt32() == 1u);
                    frame.SetBool(poseSet.RightFootContactHandle, reader.ReadUInt32() == 1u);
                }

                for (int i = 0; i < nTags; i++)
                {
                    string name = reader.ReadString();
                    int nRanges = (int)reader.ReadUInt32();
                    List<int> tagStarts = new List<int>(nRanges);
                    List<int> tagEnds = new List<int>(nRanges);
                    for (int r = 0; r < nRanges; r++)
                    {
                        tagStarts.Add((int)reader.ReadUInt32());
                        tagEnds.Add((int)reader.ReadUInt32());
                    }

                    poseSet.AddTagDeserialized(name, tagStarts, tagEnds);
                }

                poseSet.ConvertTagsToNativeArrays();
            }
        }

        return true;
    }
}
}