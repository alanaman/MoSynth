using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

namespace MotionMatching.Tests
{
public class PoseSetStorageTests
{
    private const float FrameTime = 1f / 30f;

    private static PoseSet BuildPoseSet(out Skeleton mmSkeleton)
    {
        mmSkeleton = MmTestData.BuildMmSkeleton();
        var poseSet = new PoseSet(maximumFramesPrediction: 0);
        poseSet.SetSkeletonFromFile(mmSkeleton);
        return poseSet;
    }

    private static PoseVector[] BuildPoses(Skeleton mmSkeleton, int count, uint seedOffset = 0)
    {
        var poses = new PoseVector[count];
        for (var i = 0; i < count; i++)
        {
            poses[i] = MmTestData.BuildRandomPose(mmSkeleton, (uint)(i + 1) + seedOffset);
            poses[i].leftFootContact = i % 2 == 0;
            poses[i].rightFootContact = i % 3 == 0;
        }

        return poses;
    }

    private static void AssertPoseEqual(PoseVector expected, PoseVector actual, int jointCount)
    {
        for (var i = 0; i < jointCount; i++)
        {
            Assert.AreEqual(expected.jointLocalPositions[i], actual.jointLocalPositions[i]);
            Assert.AreEqual(expected.jointLocalRotations[i].value, actual.jointLocalRotations[i].value);
            Assert.AreEqual(expected.jointLocalVelocities[i], actual.jointLocalVelocities[i]);
            Assert.AreEqual(expected.jointLocalAngularVelocities[i], actual.jointLocalAngularVelocities[i]);
        }

        Assert.AreEqual(expected.leftFootContact, actual.leftFootContact);
        Assert.AreEqual(expected.rightFootContact, actual.rightFootContact);
    }

    [Test]
    public void AddClipThenGetPose_RoundTripsExactly()
    {
        var poseSet = BuildPoseSet(out var mmSkeleton);
        var poses = BuildPoses(mmSkeleton, 10);

        Assert.IsTrue(poseSet.AddClip(poses, FrameTime, new List<AnnotatedAnimationClip.Tag>()));
        Assert.AreEqual(poses.Length, poseSet.NumberPoses);

        for (var i = 0; i < poses.Length; i++)
        {
            poseSet.GetPose(i, out var actual);
            AssertPoseEqual(poses[i], actual, mmSkeleton.Joints.Count);
        }
    }

    [Test]
    public void StorageGrowth_PreservesEarlierPoses()
    {
        var poseSet = BuildPoseSet(out var mmSkeleton);
        var firstClip = BuildPoses(mmSkeleton, 50);
        var secondClip = BuildPoses(mmSkeleton, 80, seedOffset: 1000);

        poseSet.AddClip(firstClip, FrameTime, new List<AnnotatedAnimationClip.Tag>());
        poseSet.AddClip(secondClip, FrameTime, new List<AnnotatedAnimationClip.Tag>());

        Assert.AreEqual(firstClip.Length + secondClip.Length, poseSet.NumberPoses);
        poseSet.GetPose(0, out var first);
        AssertPoseEqual(firstClip[0], first, mmSkeleton.Joints.Count);
        poseSet.GetPose(firstClip.Length + secondClip.Length - 1, out var last);
        AssertPoseEqual(secondClip[^1], last, mmSkeleton.Joints.Count);
    }

    [Test]
    public void GetPoseBuffer_AliasesStorageAndExposesContacts()
    {
        var poseSet = BuildPoseSet(out var mmSkeleton);
        var poses = BuildPoses(mmSkeleton, 3);
        poseSet.AddClip(poses, FrameTime, new List<AnnotatedAnimationClip.Tag>());

        for (var i = 0; i < poses.Length; i++)
        {
            var frame = poseSet.GetPoseBuffer(i);
            var rotations = frame.Rotations;
            for (var j = 0; j < mmSkeleton.Joints.Count; j++)
            {
                Assert.AreEqual(poses[i].jointLocalRotations[j].value, rotations[j].value);
            }

            Assert.AreEqual(poses[i].leftFootContact, frame.GetBool(poseSet.LeftFootContactHandle));
            Assert.AreEqual(poses[i].rightFootContact, frame.GetBool(poseSet.RightFootContactHandle));
        }
    }

    [Test]
    public void SkeletonAssetMirror_MatchesMmSkeleton()
    {
        var poseSet = BuildPoseSet(out var mmSkeleton);
        var asset = poseSet.SkeletonAsset;

        Assert.IsNotNull(asset);
        Assert.AreEqual(mmSkeleton.Joints.Count, asset.BoneCount);
        for (var i = 0; i < asset.BoneCount; i++)
        {
            var bone = asset.GetBone(i);
            var joint = mmSkeleton.Joints[i];
            Assert.AreEqual(joint.name, bone.name);
            Assert.AreEqual(i == 0 ? -1 : joint.parentIndex, bone.parentIndex);
            Assert.AreEqual((Vector3)joint.localOffset, (Vector3)bone.restLocalPosition);
            Assert.AreEqual(joint.type, bone.humanBone);
        }
    }

    [Test]
    public void ContactHandles_AreDistinctEvenWithoutRightSideBones()
    {
        // MmTestData's skeleton has left-side bones only, forcing the right contact channel
        // onto a fallback bone; both handles must still resolve and stay independent.
        var poseSet = BuildPoseSet(out var mmSkeleton);
        var pose = MmTestData.BuildRandomPose(mmSkeleton, 5u);
        pose.leftFootContact = true;
        pose.rightFootContact = false;
        poseSet.AddClip(pose);

        var frame = poseSet.GetPoseBuffer(0);
        Assert.IsTrue(poseSet.LeftFootContactHandle.IsValid);
        Assert.IsTrue(poseSet.RightFootContactHandle.IsValid);
        Assert.IsTrue(frame.GetBool(poseSet.LeftFootContactHandle));
        Assert.IsFalse(frame.GetBool(poseSet.RightFootContactHandle));

        frame.SetBool(poseSet.RightFootContactHandle, true);
        Assert.IsTrue(frame.GetBool(poseSet.LeftFootContactHandle));
        Assert.IsTrue(frame.GetBool(poseSet.RightFootContactHandle));
    }
}
}
