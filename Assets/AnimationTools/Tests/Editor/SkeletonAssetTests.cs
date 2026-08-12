using System;
using System.Collections.Generic;
using AnimationTools;
using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;

namespace AnimationTools.Tests
{
public class SkeletonAssetTests
{
    private SkeletonAsset skeleton;

    [TearDown]
    public void TearDown()
    {
        if (skeleton != null) UnityEngine.Object.DestroyImmediate(skeleton);
        skeleton = null;
    }

    private static Bone MakeBone(int id, int parentIndex, string name = "bone")
    {
        return new Bone
        {
            id = id, name = name, parentIndex = parentIndex,
            restLocalPosition = float3.zero, restLocalRotation = quaternion.identity,
            humanBone = HumanBodyBones.LastBone
        };
    }

    [Test]
    public void SetBones_RejectsEmptyList()
    {
        skeleton = ScriptableObject.CreateInstance<SkeletonAsset>();
        Assert.Throws<ArgumentException>(() => skeleton.SetBones(new List<Bone>(), 1));
    }

    [Test]
    public void SetBones_RejectsNonRootAtIndexZero()
    {
        skeleton = ScriptableObject.CreateInstance<SkeletonAsset>();
        var bones = new List<Bone> { MakeBone(1, 0) };
        Assert.Throws<ArgumentException>(() => skeleton.SetBones(bones, 2));
    }

    [Test]
    public void SetBones_RejectsDuplicateIds()
    {
        skeleton = ScriptableObject.CreateInstance<SkeletonAsset>();
        var bones = new List<Bone> { MakeBone(1, -1), MakeBone(1, 0) };
        Assert.Throws<ArgumentException>(() => skeleton.SetBones(bones, 3));
    }

    [Test]
    public void SetBones_RejectsNonPositiveIds()
    {
        skeleton = ScriptableObject.CreateInstance<SkeletonAsset>();
        var bones = new List<Bone> { MakeBone(0, -1) };
        Assert.Throws<ArgumentException>(() => skeleton.SetBones(bones, 1));
    }

    [Test]
    public void SetBones_RejectsParentIndexAtOrAfterOwnIndex()
    {
        skeleton = ScriptableObject.CreateInstance<SkeletonAsset>();
        var bones = new List<Bone> { MakeBone(1, -1), MakeBone(2, 1) };
        Assert.Throws<ArgumentException>(() => skeleton.SetBones(bones, 3));
    }

    [Test]
    public void SetBones_RejectsNegativeParentIndexOnNonRoot()
    {
        skeleton = ScriptableObject.CreateInstance<SkeletonAsset>();
        var bones = new List<Bone> { MakeBone(1, -1), MakeBone(2, -1) };
        Assert.Throws<ArgumentException>(() => skeleton.SetBones(bones, 3));
    }

    [Test]
    public void SetBones_RejectsNextBoneIdNotGreaterThanMaxId()
    {
        skeleton = ScriptableObject.CreateInstance<SkeletonAsset>();
        var bones = new List<Bone> { MakeBone(1, -1), MakeBone(2, 0) };
        Assert.Throws<ArgumentException>(() => skeleton.SetBones(bones, 2));
    }

    [Test]
    public void SetBones_AcceptsValidChain_AndPopulatesQueries()
    {
        skeleton = TestSkeletons.CreateChain3();

        Assert.AreEqual(3, skeleton.BoneCount);
        Assert.AreEqual(TestSkeletons.RootId, skeleton.GetBone(0).id);
        Assert.AreEqual(TestSkeletons.SpineId, skeleton.GetBone(1).id);
        Assert.AreEqual(TestSkeletons.HeadId, skeleton.GetBone(2).id);
        Assert.AreEqual(1, skeleton.IndexOfId(TestSkeletons.SpineId));
        Assert.AreEqual(-1, skeleton.IndexOfId(999));
    }

    [Test]
    public void Version_IncrementsPerSetBonesCall()
    {
        skeleton = ScriptableObject.CreateInstance<SkeletonAsset>();
        var before = skeleton.Version;

        skeleton.SetBones(new List<Bone> { MakeBone(1, -1) }, 2);
        var afterFirst = skeleton.Version;

        skeleton.SetBones(new List<Bone> { MakeBone(1, -1), MakeBone(2, 0) }, 3);
        var afterSecond = skeleton.Version;

        Assert.AreEqual(before + 1, afterFirst);
        Assert.AreEqual(afterFirst + 1, afterSecond);
    }

    [Test]
    public void GetSkeletonData_MatchesDeclaredHierarchy()
    {
        skeleton = TestSkeletons.CreateChain3();
        var data = skeleton.GetSkeletonData();

        Assert.AreEqual(3, data.BoneCount);
        Assert.AreEqual(-1, data.ParentIndices[0]);
        Assert.AreEqual(0, data.ParentIndices[1]);
        Assert.AreEqual(1, data.ParentIndices[2]);
    }

    [Test]
    public void GetSkeletonData_IsCachedUntilVersionChanges_ThenReflectsNewHierarchy()
    {
        skeleton = ScriptableObject.CreateInstance<SkeletonAsset>();
        skeleton.SetBones(new List<Bone> { MakeBone(1, -1), MakeBone(2, 0) }, 3);

        var data1 = skeleton.GetSkeletonData();
        var data2 = skeleton.GetSkeletonData();

        // Same Version -> same content each call.
        Assert.AreEqual(data1.BoneCount, data2.BoneCount);
        for (var i = 0; i < data1.BoneCount; i++)
            Assert.AreEqual(data1.ParentIndices[i], data2.ParentIndices[i]);

        skeleton.SetBones(new List<Bone> { MakeBone(1, -1), MakeBone(2, 0), MakeBone(3, 1) }, 4);
        var data3 = skeleton.GetSkeletonData();

        Assert.AreEqual(3, data3.BoneCount);
        Assert.AreEqual(1, data3.ParentIndices[2]);
    }

    [Test]
    public void BoneReference_Default_IsNotSet()
    {
        var reference = default(BoneReference);
        Assert.IsFalse(reference.IsSet);
    }

    [Test]
    public void BoneReference_ResolveIndex_NegativeOneOnNullSkeleton()
    {
        var reference = new BoneReference(TestSkeletons.RootId, "root");
        Assert.AreEqual(-1, reference.ResolveIndex(null));
    }

    [Test]
    public void BoneReference_ResolveIndex_NegativeOneWhenUnset()
    {
        skeleton = TestSkeletons.CreateChain3();
        var reference = default(BoneReference);
        Assert.AreEqual(-1, reference.ResolveIndex(skeleton));
    }

    [Test]
    public void BoneReference_ResolveIndex_NegativeOneWhenIdMissing()
    {
        skeleton = TestSkeletons.CreateChain3();
        var reference = new BoneReference(999, "ghost");
        Assert.AreEqual(-1, reference.ResolveIndex(skeleton));
    }

    [Test]
    public void BoneReference_ResolveIndex_ReturnsIndexOnHit()
    {
        skeleton = TestSkeletons.CreateChain3();
        var reference = new BoneReference(TestSkeletons.HeadId, "head");
        Assert.AreEqual(2, reference.ResolveIndex(skeleton));
    }
}
}
