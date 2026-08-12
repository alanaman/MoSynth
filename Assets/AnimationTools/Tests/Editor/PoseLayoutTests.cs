using System;
using System.Collections.Generic;
using AnimationTools;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace AnimationTools.Tests
{
public class PoseLayoutTests
{
    private SkeletonAsset skeleton;

    [SetUp]
    public void SetUp()
    {
        skeleton = TestSkeletons.CreateChain3();
    }

    [TearDown]
    public void TearDown()
    {
        if (skeleton != null) UnityEngine.Object.DestroyImmediate(skeleton);
        skeleton = null;
    }

    // Positions: root, head (2). Rotations: root, spine, head, Quaternion (3). Scale: spine (1).
    // Velocities: root, head (2). AngularVelocity: spine, RotationVector (1). Bools: spine
    // Default, head Contact (2).
    private List<ChannelDescriptor> BuildMixedChannels(RotationRepresentation rotationRepresentation = RotationRepresentation.Quaternion)
    {
        return new List<ChannelDescriptor>
        {
            new() { boneId = TestSkeletons.RootId, type = ChannelType.Position },
            new() { boneId = TestSkeletons.HeadId, type = ChannelType.Position },
            new() { boneId = TestSkeletons.RootId, type = ChannelType.Rotation, representation = rotationRepresentation },
            new() { boneId = TestSkeletons.SpineId, type = ChannelType.Rotation, representation = rotationRepresentation },
            new() { boneId = TestSkeletons.HeadId, type = ChannelType.Rotation, representation = rotationRepresentation },
            new() { boneId = TestSkeletons.SpineId, type = ChannelType.Scale },
            new() { boneId = TestSkeletons.RootId, type = ChannelType.Velocity },
            new() { boneId = TestSkeletons.HeadId, type = ChannelType.Velocity },
            new() { boneId = TestSkeletons.SpineId, type = ChannelType.AngularVelocity, representation = RotationRepresentation.RotationVector },
            new() { boneId = TestSkeletons.SpineId, type = ChannelType.Bool, usage = ChannelUsage.Default },
            new() { boneId = TestSkeletons.HeadId, type = ChannelType.Bool, usage = ChannelUsage.Contact }
        };
    }

    [Test]
    public void Build_MixedChannelSet_SectionOffsetsMatchHandComputedLayout()
    {
        var layout = PoseLayout.Build(skeleton, BuildMixedChannels());
        var data = layout.Data;

        Assert.AreEqual(0, data.PositionStart);
        Assert.AreEqual(2, data.PositionCount);

        Assert.AreEqual(6, data.RotationStart);
        Assert.AreEqual(3, data.RotationCount);
        Assert.AreEqual(4, data.RotationStride);

        Assert.AreEqual(18, data.ScaleStart);
        Assert.AreEqual(1, data.ScaleCount);

        Assert.AreEqual(21, data.VelocityStart);
        Assert.AreEqual(2, data.VelocityCount);

        Assert.AreEqual(27, data.AngularVelocityStart);
        Assert.AreEqual(1, data.AngularVelocityCount);

        Assert.AreEqual(30, data.BoolStart);
        Assert.AreEqual(2, data.BoolCount);

        Assert.AreEqual(32, data.FloatCount);
    }

    [Test]
    public void Build_RotationVectorRepresentation_ShrinksRotationStrideAndFloatCount()
    {
        var layout = PoseLayout.Build(skeleton, BuildMixedChannels(RotationRepresentation.RotationVector));
        var data = layout.Data;

        Assert.AreEqual(3, data.RotationStride);
        // Same channel counts as the Quaternion layout, but 3 fewer floats per rotation * 3 rotations.
        Assert.AreEqual(29, data.FloatCount);
    }

    [Test]
    public void Build_UnknownBoneId_Throws()
    {
        var channels = new List<ChannelDescriptor> { new() { boneId = 999, type = ChannelType.Position } };
        Assert.Throws<ArgumentException>(() => PoseLayout.Build(skeleton, channels));
    }

    [Test]
    public void Build_DuplicateTriple_Throws()
    {
        var channels = new List<ChannelDescriptor>
        {
            new() { boneId = TestSkeletons.RootId, type = ChannelType.Position, usage = ChannelUsage.Default },
            new() { boneId = TestSkeletons.RootId, type = ChannelType.Position, usage = ChannelUsage.Default }
        };
        Assert.Throws<ArgumentException>(() => PoseLayout.Build(skeleton, channels));
    }

    [Test]
    public void Build_MixedRotationRepresentations_Throws()
    {
        var channels = new List<ChannelDescriptor>
        {
            new() { boneId = TestSkeletons.RootId, type = ChannelType.Rotation, representation = RotationRepresentation.Quaternion },
            new() { boneId = TestSkeletons.SpineId, type = ChannelType.Rotation, representation = RotationRepresentation.RotationVector }
        };
        Assert.Throws<ArgumentException>(() => PoseLayout.Build(skeleton, channels));
    }

    [Test]
    public void Build_AngularVelocityWithNonRotationVectorRepresentation_Throws()
    {
        var channels = new List<ChannelDescriptor>
        {
            new() { boneId = TestSkeletons.RootId, type = ChannelType.AngularVelocity, representation = RotationRepresentation.Quaternion }
        };
        Assert.Throws<ArgumentException>(() => PoseLayout.Build(skeleton, channels));
    }

    [Test]
    public void TryBind_ReturnsDistinctSlotsThatRoundTripIndependently()
    {
        var layout = PoseLayout.Build(skeleton, BuildMixedChannels());
        var rootBone = new BoneReference(TestSkeletons.RootId, "root");
        var headBone = new BoneReference(TestSkeletons.HeadId, "head");

        Assert.IsTrue(layout.TryBindPosition(rootBone, out var rootHandle));
        Assert.IsTrue(layout.TryBindPosition(headBone, out var headHandle));

        var buffer = PoseBuffer.Allocate(layout, Allocator.Temp);
        try
        {
            var rootValue = new float3(1f, 2f, 3f);
            var headValue = new float3(4f, 5f, 6f);

            buffer.SetPosition(rootHandle, rootValue);
            buffer.SetPosition(headHandle, headValue);

            // If both handles resolved to the same element, the second write would clobber the first.
            Assert.AreEqual(rootValue, buffer.GetPosition(rootHandle));
            Assert.AreEqual(headValue, buffer.GetPosition(headHandle));

            // Declaration order (root, then head) fixes which section element each bone owns.
            Assert.AreEqual(TestSkeletons.RootId, layout.GetChannel(ChannelType.Position, 0).boneId);
            Assert.AreEqual(TestSkeletons.HeadId, layout.GetChannel(ChannelType.Position, 1).boneId);
        }
        finally
        {
            buffer.Dispose();
        }
    }

    [Test]
    public void TryBind_UndeclaredChannel_ReturnsFalseWithInvalidHandle()
    {
        var channels = new List<ChannelDescriptor> { new() { boneId = TestSkeletons.RootId, type = ChannelType.Position } };
        var layout = PoseLayout.Build(skeleton, channels);

        var found = layout.TryBindScale(new BoneReference(TestSkeletons.RootId, "root"), out var handle);

        Assert.IsFalse(found);
        Assert.IsFalse(handle.IsValid);
    }

    [Test]
    public void Bind_UndeclaredChannel_Throws()
    {
        var channels = new List<ChannelDescriptor> { new() { boneId = TestSkeletons.RootId, type = ChannelType.Position } };
        var layout = PoseLayout.Build(skeleton, channels);

        Assert.Throws<ArgumentException>(() => layout.BindScale(new BoneReference(TestSkeletons.RootId, "root")));
    }

    [Test]
    public void TryBindBool_UsageDisambiguatesSameBone()
    {
        var channels = new List<ChannelDescriptor>
        {
            new() { boneId = TestSkeletons.HeadId, type = ChannelType.Bool, usage = ChannelUsage.Default },
            new() { boneId = TestSkeletons.HeadId, type = ChannelType.Bool, usage = ChannelUsage.Contact }
        };
        var layout = PoseLayout.Build(skeleton, channels);
        var headBone = new BoneReference(TestSkeletons.HeadId, "head");

        Assert.IsTrue(layout.TryBindBool(headBone, out var defaultHandle, ChannelUsage.Default));
        Assert.IsTrue(layout.TryBindBool(headBone, out var contactHandle, ChannelUsage.Contact));

        var buffer = PoseBuffer.Allocate(layout, Allocator.Temp);
        try
        {
            buffer.SetBool(defaultHandle, true);

            // Distinct elements: setting only the Default-usage bool must not affect Contact.
            Assert.IsTrue(buffer.GetBool(defaultHandle));
            Assert.IsFalse(buffer.GetBool(contactHandle));
        }
        finally
        {
            buffer.Dispose();
        }
    }

    [Test]
    public void CreateFullPose_PositionAndRotationCountsMatchBoneCount()
    {
        var layout = PoseLayout.CreateFullPose(skeleton, false, false);

        Assert.AreEqual(skeleton.BoneCount, layout.Data.PositionCount);
        Assert.AreEqual(skeleton.BoneCount, layout.Data.RotationCount);
        Assert.AreEqual(0, layout.Data.VelocityCount);
        Assert.AreEqual(0, layout.Data.AngularVelocityCount);
    }

    [Test]
    public void CreateFullPose_ElementIndexEqualsBoneIndex()
    {
        var layout = PoseLayout.CreateFullPose(skeleton, false, false);

        for (var i = 0; i < skeleton.BoneCount; i++)
        {
            var expectedBoneId = skeleton.GetBone(i).id;
            Assert.AreEqual(expectedBoneId, layout.GetChannel(ChannelType.Position, i).boneId);
            Assert.AreEqual(expectedBoneId, layout.GetChannel(ChannelType.Rotation, i).boneId);
        }
    }

    [Test]
    public void CreateFullPose_IncludeVelocities_AddsBoneAlignedVelocitySection()
    {
        var layout = PoseLayout.CreateFullPose(skeleton, true, false);

        Assert.AreEqual(skeleton.BoneCount, layout.Data.VelocityCount);
        Assert.AreEqual(0, layout.Data.AngularVelocityCount);
        for (var i = 0; i < skeleton.BoneCount; i++)
            Assert.AreEqual(skeleton.GetBone(i).id, layout.GetChannel(ChannelType.Velocity, i).boneId);
    }

    [Test]
    public void CreateFullPose_IncludeAngularVelocities_AddsBoneAlignedAngularVelocitySection()
    {
        var layout = PoseLayout.CreateFullPose(skeleton, false, true);

        Assert.AreEqual(0, layout.Data.VelocityCount);
        Assert.AreEqual(skeleton.BoneCount, layout.Data.AngularVelocityCount);
        for (var i = 0; i < skeleton.BoneCount; i++)
            Assert.AreEqual(skeleton.GetBone(i).id, layout.GetChannel(ChannelType.AngularVelocity, i).boneId);
    }

    [Test]
    public void LayoutHash_IdenticalInputs_ProduceIdenticalHashAndCachedInstance()
    {
        var layoutA = PoseLayout.Build(skeleton, BuildMixedChannels());
        var layoutB = PoseLayout.Build(skeleton, BuildMixedChannels());

        Assert.AreEqual(layoutA.LayoutHash, layoutB.LayoutHash);
        Assert.IsTrue(ReferenceEquals(layoutA, layoutB));
    }

    [Test]
    public void LayoutHash_DifferentChannelList_ProducesDifferentHash()
    {
        var layoutA = PoseLayout.Build(skeleton, BuildMixedChannels());
        var layoutC = PoseLayout.CreateFullPose(skeleton, false, false);

        Assert.AreNotEqual(layoutA.LayoutHash, layoutC.LayoutHash);
    }
}
}
