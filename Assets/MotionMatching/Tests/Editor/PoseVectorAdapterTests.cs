using System;
using System.Collections.Generic;
using AnimationTools;
using NUnit.Framework;
using Unity.Collections;
using UnityEngine;

namespace MotionMatching.Tests
{
public class PoseVectorAdapterTests
{
    private Skeleton mmSkeleton;
    private SkeletonAsset asset;
    private SkeletonMap map;

    [SetUp]
    public void SetUp()
    {
        mmSkeleton = MmTestData.BuildMmSkeleton();
        asset = MmTestData.BuildSkeletonAsset(mmSkeleton);
        map = SkeletonMap.Build(asset, asset);
    }

    [TearDown]
    public void TearDown()
    {
        if (asset != null) UnityEngine.Object.DestroyImmediate(asset);
        asset = null;
    }

    [Test]
    public void WriteThenRead_RoundTripsExactly()
    {
        var adapter = new PoseVectorAdapter(asset, map);
        var source = MmTestData.BuildRandomPose(mmSkeleton, 7u);

        var buffer = PoseBuffer.Allocate(adapter.Layout, Allocator.Temp);
        try
        {
            adapter.Write(source, buffer);

            var destination = new PoseVector(mmSkeleton.Joints.Count) { leftFootContact = true, rightFootContact = true };
            adapter.Read(buffer, ref destination);

            AssertPoseEqual(source, destination, mmSkeleton.Joints.Count);

            // Contacts intentionally don't cross the adapter -- they must stay as initialized.
            Assert.IsTrue(destination.leftFootContact);
            Assert.IsTrue(destination.rightFootContact);
        }
        finally
        {
            buffer.Dispose();
        }
    }

    [Test]
    public void Layout_HasOnePositionRotationVelocityAngularVelocityPerBone()
    {
        var adapter = new PoseVectorAdapter(asset, map);
        var boneCount = asset.BoneCount;

        Assert.AreEqual(boneCount, adapter.Layout.Data.PositionCount);
        Assert.AreEqual(boneCount, adapter.Layout.Data.RotationCount);
        Assert.AreEqual(boneCount, adapter.Layout.Data.VelocityCount);
        Assert.AreEqual(boneCount, adapter.Layout.Data.AngularVelocityCount);
    }

    [Test]
    public void Layout_Bone0VelocityChannel_IsRootMotionRootLocal()
    {
        var adapter = new PoseVectorAdapter(asset, map);
        var channel = adapter.Layout.GetChannel(ChannelType.Velocity, 0);

        Assert.AreEqual(ChannelUsage.RootMotion, channel.usage);
        Assert.AreEqual(ChannelSpace.RootLocal, channel.space);
    }

    [Test]
    public void Layout_ExtraChannels_AreReachable()
    {
        var rootBoneId = asset.GetBone(0).id;
        var extraChannels = new List<ChannelDescriptor>
        {
            new() { boneId = rootBoneId, type = ChannelType.Bool, usage = ChannelUsage.Contact }
        };

        var adapter = new PoseVectorAdapter(asset, map, extraChannels);
        var bone = new BoneReference(rootBoneId, asset.GetBone(0).name);

        Assert.IsTrue(adapter.Layout.TryBindBool(bone, out var handle, ChannelUsage.Contact));
        Assert.IsTrue(handle.IsValid);
    }

    [Test]
    public void Constructor_MismatchedMap_ThrowsArgumentException()
    {
        var smallerMmSkeleton = MmTestData.BuildMmSkeleton();
        smallerMmSkeleton.Joints.RemoveAt(smallerMmSkeleton.Joints.Count - 1);
        var smallerAsset = MmTestData.BuildSkeletonAsset(smallerMmSkeleton);
        try
        {
            // smallerMap.AssetToMm.Length == smallerAsset.BoneCount, which is one less than
            // `asset`'s bone count -- exactly the mismatch the constructor should reject.
            var smallerMap = SkeletonMap.Build(smallerAsset, smallerAsset);
            Assert.IsNotNull(smallerMap);
            Assert.Throws<ArgumentException>(() => new PoseVectorAdapter(asset, smallerMap));
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(smallerAsset);
        }
    }

    [Test]
    public void WriteThenRead_DemoPose_RoundTripsExactly()
    {
        if (!MmTestData.TryLoadDemoPose(out var poseSet, out var poseSetSkeleton))
        {
            Assert.Ignore("Demo MotionMatchingData is not available in this environment.");
            return;
        }

        var demoSkeleton = MmTestData.BuildMmSkeletonFromAsset(poseSetSkeleton);
        var demoAsset = MmTestData.BuildSkeletonAsset(demoSkeleton);
        try
        {
            var demoMap = SkeletonMap.Build(demoAsset, demoAsset);
            var demoAdapter = new PoseVectorAdapter(demoAsset, demoMap);

            poseSet.GetPose(0, out var source);

            var buffer = PoseBuffer.Allocate(demoAdapter.Layout, Allocator.Temp);
            try
            {
                demoAdapter.Write(source, buffer);

                var destination = new PoseVector(demoSkeleton.Joints.Count);
                demoAdapter.Read(buffer, ref destination);

                AssertPoseEqual(source, destination, demoSkeleton.Joints.Count);
            }
            finally
            {
                buffer.Dispose();
            }
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(demoAsset);
        }
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
    }
}
}
