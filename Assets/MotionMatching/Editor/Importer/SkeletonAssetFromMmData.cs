using System;
using System.Collections.Generic;
using System.IO;
using AnimationTools;
using AnimationTools.Editor;
using UnityEditor;
using UnityEngine;

namespace MotionMatching
{
/// <summary>
/// Converts the transient <see cref="SkeletonAsset"/> read from a Motion Matching database's
/// .mmskeleton into a persisted one, so tooling built against the new pose architecture can share
/// bone lists with existing Motion Matching data.
/// </summary>
public static class SkeletonAssetFromMmData
{
    /// <summary>
    /// Converts <paramref name="mmSkeleton"/> and writes it to <paramref name="assetPath"/>,
    /// merging into an existing <see cref="SkeletonAsset"/> there if one already exists so bone
    /// ids (and therefore any <see cref="BoneReference"/>s pointing at them) survive re-import.
    /// </summary>
    public static SkeletonAsset CreateOrUpdate(SkeletonAsset mmSkeleton, string assetPath)
    {
        var asset = AssetDatabase.LoadAssetAtPath<SkeletonAsset>(assetPath);
        var isNew = asset == null;
        if (isNew) asset = ScriptableObject.CreateInstance<SkeletonAsset>();

        Build(mmSkeleton, asset);

        if (isNew)
        {
            AssetDatabase.CreateAsset(asset, assetPath);
            AssetDatabase.SaveAssets();
        }

        return asset;
    }

    /// <summary>
    /// Converts <paramref name="source"/> into <paramref name="target"/> without touching the
    /// AssetDatabase, for transient (in-memory) skeleton assets.
    /// </summary>
    public static void Build(SkeletonAsset source, SkeletonAsset target)
    {
        SkeletonAssetAuthoring.BuildOrMerge(target, ConvertToBoneSources(source));
    }

    private static List<SkeletonAssetAuthoring.BoneSource> ConvertToBoneSources(SkeletonAsset mmSkeleton)
    {
        var source = new List<SkeletonAssetAuthoring.BoneSource>(mmSkeleton.BoneCount);

        for (var i = 0; i < mmSkeleton.BoneCount; i++)
        {
            var bone = mmSkeleton.GetBone(i);

            if (bone.parentIndex < 0 && i != 0)
            {
                throw new InvalidOperationException(
                    $"MM joint \"{bone.name}\" at index {i} has no valid parent " +
                    $"(parentIndex {bone.parentIndex}) but is not the root.");
            }

            source.Add(new SkeletonAssetAuthoring.BoneSource
            {
                name = bone.name,
                parentIndex = bone.parentIndex,
                restLocalPosition = bone.restLocalPosition,
                restLocalRotation = bone.restLocalRotation,
                humanBone = bone.humanBone
            });
        }

        return source;
    }

    [MenuItem("MotionMatching/Create Skeleton Asset From Selection")]
    private static void CreateSkeletonAssetFromSelection()
    {
        var selected = Selection.activeObject;

        SkeletonAsset mmSkeleton;
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
        else
        {
            return;
        }

        var selectedAssetPath = AssetDatabase.GetAssetPath(selected);
        var directory = Path.GetDirectoryName(selectedAssetPath);
        var outputPath = Path.Combine(directory ?? "Assets", $"{sourceName}_Skeleton.asset")
            .Replace('\\', '/');

        var result = CreateOrUpdate(mmSkeleton, outputPath);
        UnityEngine.Object.DestroyImmediate(mmSkeleton);
        Debug.Log($"[SkeletonAssetFromMmData] Wrote {result.BoneCount} bone(s) to {outputPath}.");
    }

    [MenuItem("MotionMatching/Create Skeleton Asset From Selection", true)]
    private static bool ValidateCreateSkeletonAssetFromSelection()
    {
        return Selection.activeObject is IPoseSetSource;
    }
}
}
