using System;
using System.Collections.Generic;
using System.IO;
using AnimationTools;
using AnimationTools.Editor;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;

namespace MotionMatching
{
/// <summary>
/// Converts a Motion Matching <see cref="Skeleton"/> (from a pose database or a raw BVH import)
/// into an <see cref="AnimationTools.SkeletonAsset"/>, so tooling built against the new pose
/// architecture can share bone lists with existing Motion Matching data.
/// </summary>
public static class SkeletonAssetFromMmData
{
    /// <summary>
    /// Converts <paramref name="mmSkeleton"/> and writes it to <paramref name="assetPath"/>,
    /// merging into an existing <see cref="SkeletonAsset"/> there if one already exists so bone
    /// ids (and therefore any <see cref="BoneReference"/>s pointing at them) survive re-import.
    /// </summary>
    public static SkeletonAsset CreateOrUpdate(Skeleton mmSkeleton, string assetPath)
    {
        var source = ConvertToBoneSources(mmSkeleton);

        var asset = AssetDatabase.LoadAssetAtPath<SkeletonAsset>(assetPath);
        var isNew = asset == null;
        if (isNew) asset = ScriptableObject.CreateInstance<SkeletonAsset>();

        SkeletonAssetAuthoring.BuildOrMerge(asset, source);

        if (isNew)
        {
            AssetDatabase.CreateAsset(asset, assetPath);
            AssetDatabase.SaveAssets();
        }

        return asset;
    }

    private static List<SkeletonAssetAuthoring.BoneSource> ConvertToBoneSources(Skeleton mmSkeleton)
    {
        var joints = mmSkeleton.Joints;
        var source = new List<SkeletonAssetAuthoring.BoneSource>(joints.Count);

        for (var i = 0; i < joints.Count; i++)
        {
            var joint = joints[i];

            // PoseSet.SetSkeletonFromBvh roots MM skeletons at index 0 with parentIndex -1, matching
            // AnimationTools' own root convention. Some sources instead self-reference the root
            // (parentIndex == its own index) rather than using -1; normalize both to -1, and only
            // accept either at index 0 -- anywhere else it means a genuinely missing parent.
            var parentIndex = joint.parentIndex;
            if (parentIndex == i || parentIndex < 0)
            {
                if (i != 0)
                {
                    throw new InvalidOperationException(
                        $"MM joint \"{joint.name}\" at index {i} has no valid parent " +
                        $"(parentIndex {joint.parentIndex}) but is not the root.");
                }

                parentIndex = -1;
            }

            source.Add(new SkeletonAssetAuthoring.BoneSource
            {
                name = joint.name,
                parentIndex = parentIndex,
                restLocalPosition = joint.localOffset,
                // MM skeletons carry rest offsets only, no rest rotation.
                restLocalRotation = quaternion.identity,
                humanBone = joint.type
            });
        }

        return source;
    }

    [MenuItem("MotionMatching/Create Skeleton Asset From Selection")]
    private static void CreateSkeletonAssetFromSelection()
    {
        var selected = Selection.activeObject;

        Skeleton mmSkeleton;
        string sourceName;

        if (selected is IPoseSetSource poseSetSource)
        {
            if (!PoseSerializer.TryDeserializeSkeleton(
                    poseSetSource.GetAssetPath(), poseSetSource.name, out mmSkeleton))
            {
                Debug.LogError(
                    $"[SkeletonAssetFromMmData] No .mmskeleton found for \"{poseSetSource.name}\" at " +
                    $"{poseSetSource.GetAssetPath()}. Generate the pose database first.");
                return;
            }

            sourceName = poseSetSource.name;
        }
        else if (selected is BvhAnimation bvhAnimation)
        {
            mmSkeleton = bvhAnimation.Skeleton;
            sourceName = bvhAnimation.name;
        }
        else
        {
            return;
        }

        var selectedAssetPath = AssetDatabase.GetAssetPath(selected);
        var directory = Path.GetDirectoryName(selectedAssetPath);
        var outputPath = Path.Combine(directory ?? "Assets", $"{sourceName}_Skeleton.asset")
            .Replace('\\', '/');

        var result = CreateOrUpdate(mmSkeleton, outputPath);
        Debug.Log($"[SkeletonAssetFromMmData] Wrote {result.BoneCount} bone(s) to {outputPath}.");
    }

    [MenuItem("MotionMatching/Create Skeleton Asset From Selection", true)]
    private static bool ValidateCreateSkeletonAssetFromSelection()
    {
        return Selection.activeObject is IPoseSetSource || Selection.activeObject is BvhAnimation;
    }
}
}
