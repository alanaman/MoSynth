using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;
using AnimationTools;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.TestTools;

namespace MotionMatching.Tests
{
public class PoseSetStorageTests
{
    private const float FrameTime = 1f / 30f;

    private static PoseSet BuildPoseSet(out Skeleton skeleton)
    {
        skeleton = MmTestData.BuildSkeleton();
        var poseSet = new PoseSet();
        poseSet.SetSkeleton(skeleton);
        return poseSet;
    }

    private static void AssertPoseEqual(PoseBuffer expected, PoseBuffer actual, int jointCount)
    {
        var expectedPositions = expected.Positions;
        var actualPositions = actual.Positions;
        var expectedRotations = expected.Rotations;
        var actualRotations = actual.Rotations;
        var expectedVelocities = expected.Velocities;
        var actualVelocities = actual.Velocities;
        var expectedAngularVelocities = expected.AngularVelocities;
        var actualAngularVelocities = actual.AngularVelocities;

        for (var i = 0; i < jointCount; i++)
        {
            Assert.AreEqual(expectedPositions[i], actualPositions[i]);
            Assert.AreEqual(expectedRotations[i].value, actualRotations[i].value);
            Assert.AreEqual(expectedVelocities[i], actualVelocities[i]);
            Assert.AreEqual(expectedAngularVelocities[i], actualAngularVelocities[i]);
        }
    }

    [Test]
    public void AddClipThenGetPose_RoundTripsExactly()
    {
        var poseSet = BuildPoseSet(out var skeleton);
        const int count = 10;
        var frames = poseSet.BeginClip(count, FrameTime);
        for (var i = 0; i < count; i++)
        {
            var frame = frames[i];
            MmTestData.FillRandomPose(frame, skeleton, (uint)(i + 1));
            frame.SetBool(poseSet.LeftFootContactHandle, i % 2 == 0);
            frame.SetBool(poseSet.RightFootContactHandle, i % 3 == 0);
        }

        poseSet.EndClip(new List<AnnotatedAnimationClip.Tag>());

        Assert.AreEqual(count, poseSet.NumberPoses);

        var expected = PoseBuffer.Allocate(poseSet.PoseLayout, Allocator.Temp);
        try
        {
            for (var i = 0; i < count; i++)
            {
                MmTestData.FillRandomPose(expected, skeleton, (uint)(i + 1));
                expected.SetBool(poseSet.LeftFootContactHandle, i % 2 == 0);
                expected.SetBool(poseSet.RightFootContactHandle, i % 3 == 0);

                var actual = poseSet.GetPoseBuffer(i);
                AssertPoseEqual(expected, actual, skeleton.BoneCount);
                Assert.AreEqual(expected.GetBool(poseSet.LeftFootContactHandle), actual.GetBool(poseSet.LeftFootContactHandle));
                Assert.AreEqual(expected.GetBool(poseSet.RightFootContactHandle), actual.GetBool(poseSet.RightFootContactHandle));
            }
        }
        finally
        {
            expected.Dispose();
        }
    }

    [Test]
    public void StorageGrowth_PreservesEarlierPoses()
    {
        var poseSet = BuildPoseSet(out var skeleton);
        const int firstCount = 50;
        const int secondCount = 80;
        const uint secondSeedOffset = 1000;

        var firstFrames = poseSet.BeginClip(firstCount, FrameTime);
        for (var i = 0; i < firstCount; i++)
        {
            var frame = firstFrames[i];
            MmTestData.FillRandomPose(frame, skeleton, (uint)(i + 1));
            frame.SetBool(poseSet.LeftFootContactHandle, i % 2 == 0);
            frame.SetBool(poseSet.RightFootContactHandle, i % 3 == 0);
        }

        poseSet.EndClip(new List<AnnotatedAnimationClip.Tag>());

        var secondFrames = poseSet.BeginClip(secondCount, FrameTime);
        for (var i = 0; i < secondCount; i++)
        {
            var frame = secondFrames[i];
            MmTestData.FillRandomPose(frame, skeleton, (uint)(i + 1) + secondSeedOffset);
            frame.SetBool(poseSet.LeftFootContactHandle, i % 2 == 0);
            frame.SetBool(poseSet.RightFootContactHandle, i % 3 == 0);
        }

        poseSet.EndClip(new List<AnnotatedAnimationClip.Tag>());

        Assert.AreEqual(firstCount + secondCount, poseSet.NumberPoses);

        var expected = PoseBuffer.Allocate(poseSet.PoseLayout, Allocator.Temp);
        try
        {
            MmTestData.FillRandomPose(expected, skeleton, 1u);
            expected.SetBool(poseSet.LeftFootContactHandle, true);
            expected.SetBool(poseSet.RightFootContactHandle, true);
            var first = poseSet.GetPoseBuffer(0);
            AssertPoseEqual(expected, first, skeleton.BoneCount);

            var lastLocalIndex = secondCount - 1;
            MmTestData.FillRandomPose(expected, skeleton, (uint)(lastLocalIndex + 1) + secondSeedOffset);
            expected.SetBool(poseSet.LeftFootContactHandle, lastLocalIndex % 2 == 0);
            expected.SetBool(poseSet.RightFootContactHandle, lastLocalIndex % 3 == 0);
            var last = poseSet.GetPoseBuffer(firstCount + secondCount - 1);
            AssertPoseEqual(expected, last, skeleton.BoneCount);
        }
        finally
        {
            expected.Dispose();
        }
    }

    [Test]
    public void GetPoseBuffer_AliasesStorageAndExposesContacts()
    {
        var poseSet = BuildPoseSet(out var skeleton);
        const int count = 3;
        var frames = poseSet.BeginClip(count, FrameTime);
        for (var i = 0; i < count; i++)
        {
            var frame = frames[i];
            MmTestData.FillRandomPose(frame, skeleton, (uint)(i + 1));
            frame.SetBool(poseSet.LeftFootContactHandle, i % 2 == 0);
            frame.SetBool(poseSet.RightFootContactHandle, i % 3 == 0);
        }

        poseSet.EndClip(new List<AnnotatedAnimationClip.Tag>());

        var expected = PoseBuffer.Allocate(poseSet.PoseLayout, Allocator.Temp);
        try
        {
            for (var i = 0; i < count; i++)
            {
                MmTestData.FillRandomPose(expected, skeleton, (uint)(i + 1));

                var frame = poseSet.GetPoseBuffer(i);
                var rotations = frame.Rotations;
                var expectedRotations = expected.Rotations;
                for (var j = 0; j < skeleton.BoneCount; j++)
                {
                    Assert.AreEqual(expectedRotations[j].value, rotations[j].value);
                }

                Assert.AreEqual(i % 2 == 0, frame.GetBool(poseSet.LeftFootContactHandle));
                Assert.AreEqual(i % 3 == 0, frame.GetBool(poseSet.RightFootContactHandle));
            }
        }
        finally
        {
            expected.Dispose();
        }
    }

    [Test]
    public void SetSkeletonFromBvh_PrependsSimulationBone()
    {
        // BVH-style skeleton: the mm test skeleton minus its SimulationBone, indices unshifted.
        var bvhBones = new List<SkeletonBoneData>
        {
            new() { name = "Hips", parentIndex = -1, restLocalPosition = new float3(0f, 1f, 0f), restLocalRotation = quaternion.identity },
            new() { name = "Spine", parentIndex = 0, restLocalPosition = new float3(0f, 0.2f, 0f), restLocalRotation = quaternion.identity },
            new() { name = "LeftFoot", parentIndex = 0, restLocalPosition = new float3(0.2f, -0.9f, 0f), restLocalRotation = quaternion.identity },
            new() { name = "LeftToe", parentIndex = 2, restLocalPosition = new float3(0f, -0.1f, 0.15f), restLocalRotation = quaternion.identity }
        };
        var bvhSkeleton = new Skeleton(bvhBones, "BvhSkeleton");

        var poseSet = new PoseSet();
        poseSet.SetSkeletonFromBvh(bvhSkeleton);
        var skeleton = poseSet.Skeleton;
        var expected = MmTestData.BuildSkeleton();

        Assert.IsNotNull(skeleton);
        Assert.AreEqual(expected.BoneCount, skeleton.BoneCount);
        for (var i = 0; i < skeleton.BoneCount; i++)
        {
            var bone = skeleton.GetBone(i);
            var expectedBone = expected.GetBone(i);
            Assert.AreEqual(expectedBone.name, bone.name);
            Assert.AreEqual(expectedBone.parentIndex, bone.parentIndex);
            Assert.AreEqual((Vector3)expectedBone.restLocalPosition, (Vector3)bone.restLocalPosition);
            Assert.AreEqual(quaternion.identity.value, bone.restLocalRotation.value);
        }
    }

    [Test]
    public void ContactHandles_AreDistinctEvenWithoutRightSideBones()
    {
        // MmTestData's skeleton has left-side bones only, forcing the right contact channel
        // onto a fallback bone; both handles must still resolve and stay independent.
        var poseSet = BuildPoseSet(out var skeleton);
        var frames = poseSet.BeginClip(1, FrameTime);
        var frame = frames[0];
        MmTestData.FillRandomPose(frame, skeleton, 5u);
        frame.SetBool(poseSet.LeftFootContactHandle, true);
        frame.SetBool(poseSet.RightFootContactHandle, false);
        poseSet.EndClip(new List<AnnotatedAnimationClip.Tag>());

        Assert.IsTrue(poseSet.LeftFootContactHandle.IsValid);
        Assert.IsTrue(poseSet.RightFootContactHandle.IsValid);
        Assert.IsTrue(frame.GetBool(poseSet.LeftFootContactHandle));
        Assert.IsFalse(frame.GetBool(poseSet.RightFootContactHandle));

        frame.SetBool(poseSet.RightFootContactHandle, true);
        Assert.IsTrue(frame.GetBool(poseSet.LeftFootContactHandle));
        Assert.IsTrue(frame.GetBool(poseSet.RightFootContactHandle));
    }

    [Test]
    public void Layout_SharedBetweenBuilderCalls()
    {
        var skeleton = MmTestData.BuildSkeleton();

        var first = PoseLayoutBuilder.Build(skeleton, out _);
        var second = PoseLayoutBuilder.Build(skeleton, out _);
        Assert.AreSame(first, second);

        var poseSet = new PoseSet();
        poseSet.SetSkeleton(skeleton);
        Assert.AreSame(first, poseSet.PoseLayout);
    }

    [Test]
    public void Serialize_ThenTryDeserializeSkeleton_RoundTripsRestRotations()
    {
        var bones = new List<SkeletonBoneData>
        {
            new() { name = "root", parentIndex = -1, restLocalPosition = float3.zero, restLocalRotation = quaternion.identity },
            new()
            {
                name = "spine", parentIndex = 0, restLocalPosition = new float3(0f, 1f, 0f),
                restLocalRotation = quaternion.AxisAngle(math.normalize(new float3(0.3f, 1f, -0.2f)), 1.1f)
            }
        };
        var skeleton = new Skeleton(bones, "RotationRoundTrip");

        var poseSet = new PoseSet();
        poseSet.SetSkeleton(skeleton);

        var tempDir = Path.Combine(Application.temporaryCachePath,
            "PoseSetStorageTests_" + Path.GetRandomFileName());
        const string fileName = "RestRotationRoundTrip";

        try
        {
            new PoseSerializer().Serialize(poseSet, tempDir, fileName);

            Assert.IsTrue(PoseSerializer.TryDeserializeSkeleton(tempDir, fileName, out var loaded));
            Assert.AreEqual(skeleton.BoneCount, loaded.BoneCount);

            for (var i = 0; i < skeleton.BoneCount; i++)
            {
                var expectedRotation = skeleton.GetBone(i).restLocalRotation.value;
                var actualRotation = loaded.GetBone(i).restLocalRotation.value;
                // q and -q represent the same rotation; align hemispheres before comparing.
                if (math.dot(expectedRotation, actualRotation) < 0f) actualRotation = -actualRotation;

                Assert.AreEqual(expectedRotation.x, actualRotation.x, 1e-6f);
                Assert.AreEqual(expectedRotation.y, actualRotation.y, 1e-6f);
                Assert.AreEqual(expectedRotation.z, actualRotation.z, 1e-6f);
                Assert.AreEqual(expectedRotation.w, actualRotation.w, 1e-6f);
            }
        }
        finally
        {
            if (Directory.Exists(tempDir)) Directory.Delete(tempDir, true);
            poseSet.Dispose();
        }
    }

    [Test]
    public void TryDeserializeSkeleton_WrongMagic_ReturnsFalseAndLogsError()
    {
        var tempDir = Path.Combine(Application.temporaryCachePath,
            "PoseSetStorageTests_" + Path.GetRandomFileName());
        const string fileName = "NotASkeleton";

        try
        {
            Directory.CreateDirectory(tempDir);
            File.WriteAllBytes(Path.Combine(tempDir, fileName + ".mmskeleton"), new byte[] { 0, 1, 2, 3, 4, 5, 6, 7 });

            LogAssert.Expect(LogType.Error, new Regex("old or unknown format"));
            Assert.IsFalse(PoseSerializer.TryDeserializeSkeleton(tempDir, fileName, out var skeleton));
            Assert.IsNull(skeleton);
        }
        finally
        {
            if (Directory.Exists(tempDir)) Directory.Delete(tempDir, true);
        }
    }
}
}
