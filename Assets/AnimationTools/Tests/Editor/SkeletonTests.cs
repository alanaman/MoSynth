using System;
using System.Collections.Generic;
using AnimationTools;
using NUnit.Framework;
using Unity.Mathematics;

namespace AnimationTools.Tests
{
public class SkeletonTests
{
    private static SkeletonBoneData MakeBone(string name, int parentIndex)
    {
        return new SkeletonBoneData
        {
            name = name, parentIndex = parentIndex,
            restLocalPosition = float3.zero, restLocalRotation = quaternion.identity
        };
    }

    [Test]
    public void Constructor_RejectsEmptyList()
    {
        Assert.Throws<ArgumentException>(() => new Skeleton(new List<SkeletonBoneData>()));
    }

    [Test]
    public void Constructor_RejectsNonRootAtIndexZero()
    {
        var bones = new List<SkeletonBoneData> { MakeBone("root", 0) };
        Assert.Throws<ArgumentException>(() => new Skeleton(bones));
    }

    [Test]
    public void Constructor_RejectsParentIndexAtOrAfterOwnIndex()
    {
        var bones = new List<SkeletonBoneData> { MakeBone("root", -1), MakeBone("spine", 1) };
        Assert.Throws<ArgumentException>(() => new Skeleton(bones));
    }

    [Test]
    public void Constructor_RejectsNegativeParentIndexOnNonRoot()
    {
        var bones = new List<SkeletonBoneData> { MakeBone("root", -1), MakeBone("spine", -1) };
        Assert.Throws<ArgumentException>(() => new Skeleton(bones));
    }

    [Test]
    public void Constructor_AcceptsValidChain()
    {
        var skeleton = TestSkeletons.CreateChain3();
        Assert.AreEqual(3, skeleton.BoneCount);
    }

    [Test]
    public void TryFindByName_ReturnsIndexOnHit()
    {
        var skeleton = TestSkeletons.CreateChain3();
        Assert.IsTrue(skeleton.TryFindByName("head", out var index));
        Assert.AreEqual(2, index);
    }

    [Test]
    public void TryFindByName_ReturnsFalseOnMiss()
    {
        var skeleton = TestSkeletons.CreateChain3();
        Assert.IsFalse(skeleton.TryFindByName("ghost", out var index));
        Assert.AreEqual(-1, index);
    }

    [Test]
    public void IndexOfName_MirrorsTryFindByName()
    {
        var skeleton = TestSkeletons.CreateChain3();
        Assert.AreEqual(1, skeleton.IndexOfName("spine"));
        Assert.AreEqual(-1, skeleton.IndexOfName("ghost"));
    }

    [Test]
    public void IndexOfId_ZeroIsUnsetAndReturnsNegativeOne()
    {
        var skeleton = TestSkeletons.CreateChain3();
        Assert.AreEqual(-1, skeleton.IndexOfId(0));
    }

    [Test]
    public void IndexOfId_OneBeyondBoneCountReturnsNegativeOne()
    {
        var skeleton = TestSkeletons.CreateChain3();
        Assert.AreEqual(-1, skeleton.IndexOfId(skeleton.BoneCount + 1));
    }

    [Test]
    public void IndexOfId_OneResolvesToIndexZero()
    {
        var skeleton = TestSkeletons.CreateChain3();
        Assert.AreEqual(0, skeleton.IndexOfId(1));
    }

    [Test]
    public void WithRootPrepended_AddsOneBone()
    {
        var skeleton = TestSkeletons.CreateChain3();
        var prepended = skeleton.WithRootPrepended("SimulationBone");
        Assert.AreEqual(skeleton.BoneCount + 1, prepended.BoneCount);
    }

    [Test]
    public void WithRootPrepended_NewRootIsAtIndexZeroWithNoParent()
    {
        var skeleton = TestSkeletons.CreateChain3();
        var prepended = skeleton.WithRootPrepended("SimulationBone");

        var newRoot = prepended.GetBone(0);
        Assert.AreEqual("SimulationBone", newRoot.name);
        Assert.AreEqual(-1, newRoot.parentIndex);
    }

    [Test]
    public void WithRootPrepended_OldRootBecomesChildOfNewRoot()
    {
        var skeleton = TestSkeletons.CreateChain3();
        var prepended = skeleton.WithRootPrepended("SimulationBone");

        var oldRoot = prepended.GetBone(1);
        Assert.AreEqual("root", oldRoot.name);
        Assert.AreEqual(0, oldRoot.parentIndex);
    }

    [Test]
    public void WithRootPrepended_DeeperBonesShiftParentIndexByOne()
    {
        var skeleton = TestSkeletons.CreateChain3();
        var prepended = skeleton.WithRootPrepended("SimulationBone");

        var spine = prepended.GetBone(2);
        Assert.AreEqual("spine", spine.name);
        Assert.AreEqual(1, spine.parentIndex);

        var head = prepended.GetBone(3);
        Assert.AreEqual("head", head.name);
        Assert.AreEqual(2, head.parentIndex);
    }

    [Test]
    public void WithRootPrepended_PreservesRestPoseOfShiftedBones()
    {
        var skeleton = TestSkeletons.CreateChain3();
        var prepended = skeleton.WithRootPrepended("SimulationBone");

        for (var i = 0; i < skeleton.BoneCount; i++)
        {
            var original = skeleton.GetBone(i);
            var shifted = prepended.GetBone(i + 1);
            Assert.AreEqual(original.restLocalPosition, shifted.restLocalPosition);
            Assert.AreEqual(original.restLocalRotation.value, shifted.restLocalRotation.value);
        }
    }

    [Test]
    public void ContentHash_IdenticalStructures_AreEqual()
    {
        var a = TestSkeletons.CreateChain3();
        var b = TestSkeletons.CreateChain3();
        Assert.AreEqual(a.ContentHash, b.ContentHash);
    }

    [Test]
    public void ContentHash_DifferentNames_AreLikelyDifferent()
    {
        var chain = TestSkeletons.CreateChain3();
        var branch = TestSkeletons.CreateBranch4();
        Assert.AreNotEqual(chain.ContentHash, branch.ContentHash);
    }

    [Test]
    public void StructurallyEqual_IgnoresRestPose()
    {
        var bonesA = new List<SkeletonBoneData>
        {
            MakeBone("root", -1),
            new() { name = "spine", parentIndex = 0, restLocalPosition = new float3(0f, 1f, 0f), restLocalRotation = quaternion.identity }
        };
        var bonesB = new List<SkeletonBoneData>
        {
            MakeBone("root", -1),
            new() { name = "spine", parentIndex = 0, restLocalPosition = new float3(5f, -2f, 3f), restLocalRotation = quaternion.RotateY(1f) }
        };

        Assert.IsTrue(Skeleton.StructurallyEqual(new Skeleton(bonesA), new Skeleton(bonesB)));
    }

    [Test]
    public void StructurallyEqual_DifferentNames_ReturnsFalse()
    {
        var chain = TestSkeletons.CreateChain3();
        var branch = TestSkeletons.CreateBranch4();
        Assert.IsFalse(Skeleton.StructurallyEqual(chain, branch));
    }

    [Test]
    public void StructurallyEqual_DifferentParents_ReturnsFalse()
    {
        var bonesA = new List<SkeletonBoneData> { MakeBone("root", -1), MakeBone("a", 0), MakeBone("b", 1) };
        var bonesB = new List<SkeletonBoneData> { MakeBone("root", -1), MakeBone("a", 0), MakeBone("b", 0) };

        Assert.IsFalse(Skeleton.StructurallyEqual(new Skeleton(bonesA), new Skeleton(bonesB)));
    }

    [Test]
    public void StructurallyEqual_NullSafety()
    {
        var skeleton = TestSkeletons.CreateChain3();

        Assert.IsTrue(Skeleton.StructurallyEqual(null, null));
        Assert.IsFalse(Skeleton.StructurallyEqual(skeleton, null));
        Assert.IsFalse(Skeleton.StructurallyEqual(null, skeleton));
        Assert.IsTrue(Skeleton.StructurallyEqual(skeleton, skeleton));
    }
}
}
