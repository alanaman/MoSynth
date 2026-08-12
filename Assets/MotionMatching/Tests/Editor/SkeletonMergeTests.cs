using System;
using System.Collections.Generic;
using AnimationTools;
using AnimationTools.Editor;
using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching.Tests
{
/// <summary>Stable-id merge behavior of <see cref="SkeletonAssetAuthoring.BuildOrMerge"/>.</summary>
public class SkeletonMergeTests
{
    private SkeletonAsset asset;

    [TearDown]
    public void TearDown()
    {
        if (asset != null) UnityEngine.Object.DestroyImmediate(asset);
        asset = null;
    }

    private static SkeletonAssetAuthoring.BoneSource MakeSource(string name, int parentIndex)
    {
        return new SkeletonAssetAuthoring.BoneSource
        {
            name = name, parentIndex = parentIndex,
            restLocalPosition = float3.zero, restLocalRotation = quaternion.identity,
            humanBone = HumanBodyBones.LastBone
        };
    }

    [Test]
    public void BuildOrMerge_InitialBuild_AssignsSequentialIdsAndAdvancesNextId()
    {
        asset = ScriptableObject.CreateInstance<SkeletonAsset>();
        var source = new List<SkeletonAssetAuthoring.BoneSource>
        {
            MakeSource("root", -1),
            MakeSource("spine", 0),
            MakeSource("head", 1)
        };

        SkeletonAssetAuthoring.BuildOrMerge(asset, source);

        Assert.AreEqual(1, asset.GetBone(0).id);
        Assert.AreEqual(2, asset.GetBone(1).id);
        Assert.AreEqual(3, asset.GetBone(2).id);
        Assert.AreEqual(4, asset.NextBoneId);
    }

    [Test]
    public void BuildOrMerge_SameNames_KeepsIdsAndBumpsVersion()
    {
        asset = ScriptableObject.CreateInstance<SkeletonAsset>();
        var source = new List<SkeletonAssetAuthoring.BoneSource>
        {
            MakeSource("root", -1),
            MakeSource("spine", 0),
            MakeSource("head", 1)
        };
        SkeletonAssetAuthoring.BuildOrMerge(asset, source);
        var rootId = asset.GetBone(0).id;
        var spineId = asset.GetBone(1).id;
        var headId = asset.GetBone(2).id;
        var versionBefore = asset.Version;

        SkeletonAssetAuthoring.BuildOrMerge(asset, source);

        Assert.AreEqual(rootId, asset.GetBone(0).id);
        Assert.AreEqual(spineId, asset.GetBone(1).id);
        Assert.AreEqual(headId, asset.GetBone(2).id);
        Assert.AreEqual(versionBefore + 1, asset.Version);
    }

    [Test]
    public void BuildOrMerge_OneRemovedOneAdded_SurvivorsKeepIds_NewBoneGetsFreshId_RemovedIdAbsent()
    {
        asset = ScriptableObject.CreateInstance<SkeletonAsset>();
        var initial = new List<SkeletonAssetAuthoring.BoneSource>
        {
            MakeSource("root", -1),
            MakeSource("spine", 0),
            MakeSource("head", 1)
        };
        SkeletonAssetAuthoring.BuildOrMerge(asset, initial);
        var rootId = asset.GetBone(0).id;
        var spineId = asset.GetBone(1).id;
        var headId = asset.GetBone(2).id;

        // Remove "head", add "leftHand" in its place.
        var updated = new List<SkeletonAssetAuthoring.BoneSource>
        {
            MakeSource("root", -1),
            MakeSource("spine", 0),
            MakeSource("leftHand", 1)
        };
        SkeletonAssetAuthoring.BuildOrMerge(asset, updated);

        Assert.AreEqual(rootId, asset.GetBone(0).id);
        Assert.AreEqual(spineId, asset.GetBone(1).id);

        var newId = asset.GetBone(2).id;
        Assert.Greater(newId, headId);
        Assert.AreEqual(-1, asset.IndexOfId(headId));
        Assert.AreEqual(2, asset.IndexOfId(newId));
    }

    [Test]
    public void BuildOrMerge_ReorderedSiblings_IdsFollowNamesNotPositions()
    {
        asset = ScriptableObject.CreateInstance<SkeletonAsset>();
        var initial = new List<SkeletonAssetAuthoring.BoneSource>
        {
            MakeSource("root", -1),
            MakeSource("leftHand", 0),
            MakeSource("rightHand", 0)
        };
        SkeletonAssetAuthoring.BuildOrMerge(asset, initial);
        var leftHandId = asset.GetBone(1).id;
        var rightHandId = asset.GetBone(2).id;

        // Same two siblings, swapped order -- still a valid DFS list (both parented at index 0).
        var reordered = new List<SkeletonAssetAuthoring.BoneSource>
        {
            MakeSource("root", -1),
            MakeSource("rightHand", 0),
            MakeSource("leftHand", 0)
        };
        SkeletonAssetAuthoring.BuildOrMerge(asset, reordered);

        Assert.AreEqual(rightHandId, asset.GetBone(1).id);
        Assert.AreEqual(leftHandId, asset.GetBone(2).id);
    }

    [Test]
    public void BuildOrMerge_RenamedBone_IsTreatedAsRemovedPlusAdded()
    {
        asset = ScriptableObject.CreateInstance<SkeletonAsset>();
        var initial = new List<SkeletonAssetAuthoring.BoneSource>
        {
            MakeSource("root", -1),
            MakeSource("spine", 0)
        };
        SkeletonAssetAuthoring.BuildOrMerge(asset, initial);
        var spineId = asset.GetBone(1).id;

        var renamed = new List<SkeletonAssetAuthoring.BoneSource>
        {
            MakeSource("root", -1),
            MakeSource("spine2", 0)
        };
        SkeletonAssetAuthoring.BuildOrMerge(asset, renamed);

        // Name is the merge key, not position -- a rename is indistinguishable from a
        // remove+add, so it gets a brand-new id. Pinned here as documented behavior.
        Assert.AreNotEqual(spineId, asset.GetBone(1).id);
        Assert.Greater(asset.GetBone(1).id, spineId);
    }

    [Test]
    public void BuildOrMerge_InvalidSource_ParentIndexAtOrAfterOwnIndex_Throws()
    {
        asset = ScriptableObject.CreateInstance<SkeletonAsset>();
        var invalid = new List<SkeletonAssetAuthoring.BoneSource>
        {
            MakeSource("root", -1),
            MakeSource("spine", 1)
        };

        Assert.Throws<ArgumentException>(() => SkeletonAssetAuthoring.BuildOrMerge(asset, invalid));
    }
}
}
