using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;

namespace AnimationTools.Editor
{
/// <summary>
/// Editor-only construction of <see cref="SkeletonAsset"/> bone lists. Kept separate from the
/// runtime asset so importers producing bones from different sources (Animator, BVH, Motion
/// Matching data) can all funnel through the same stable-id merge.
/// </summary>
public static class SkeletonAssetAuthoring
{
    /// <summary>Depth-first-ordered bone description used as input to <see cref="BuildOrMerge"/>.</summary>
    public struct BoneSource
    {
        public string name;
        public int parentIndex;
        public float3 restLocalPosition;
        public quaternion restLocalRotation;
        public HumanBodyBones humanBone;
    }

    /// <summary>
    /// Writes <paramref name="source"/> into <paramref name="asset"/>, keeping the id of any bone
    /// whose name already exists on the asset so existing <see cref="BoneReference"/>s survive.
    /// Bones on the asset that no longer appear in <paramref name="source"/> are dropped; their
    /// ids are never reused, since the id counter only ever advances.
    /// </summary>
    public static void BuildOrMerge(SkeletonAsset asset, IReadOnlyList<BoneSource> source)
    {
        ValidateDepthFirstOrder(source);

        var nextId = asset.NextBoneId;
        var newBones = new List<Bone>(source.Count);

        for (var i = 0; i < source.Count; i++)
        {
            var boneSource = source[i];
            var id = asset.TryFindByName(boneSource.name, out var existingIndex)
                ? asset.GetBone(existingIndex).id
                : nextId++;

            newBones.Add(new Bone
            {
                id = id,
                name = boneSource.name,
                parentIndex = boneSource.parentIndex,
                restLocalPosition = boneSource.restLocalPosition,
                restLocalRotation = boneSource.restLocalRotation,
                humanBone = boneSource.humanBone
            });
        }

        asset.SetBones(newBones, nextId);
        EditorUtility.SetDirty(asset);
    }

    private static void ValidateDepthFirstOrder(IReadOnlyList<BoneSource> source)
    {
        if (source == null || source.Count == 0)
            throw new ArgumentException("Source bone list must contain at least one bone.", nameof(source));

        if (source[0].parentIndex != -1)
            throw new ArgumentException(
                "Source bone at index 0 must be the root (parentIndex == -1).", nameof(source));

        for (var i = 1; i < source.Count; i++)
        {
            var parentIndex = source[i].parentIndex;
            if (parentIndex < 0 || parentIndex >= i)
                throw new ArgumentException(
                    $"Source bone \"{source[i].name}\" at index {i} has parentIndex {parentIndex}, " +
                    "which violates the depth-first invariant (parentIndex must be < index).",
                    nameof(source));
        }
    }

    // --- Create from selected Animator ------------------------------------------------------

    [MenuItem("Assets/Create/AnimationTools/Skeleton From Selected Animator")]
    private static void CreateFromAnimator()
    {
        var animator = Selection.activeGameObject.GetComponent<Animator>();
        var source = BuildBoneSources(animator);

        var asset = ScriptableObject.CreateInstance<SkeletonAsset>();
        BuildOrMerge(asset, source);

        ProjectWindowUtil.CreateAsset(asset, $"{animator.gameObject.name}_Skeleton.asset");
    }

    [MenuItem("Assets/Create/AnimationTools/Skeleton From Selected Animator", true)]
    private static bool ValidateCreateFromAnimator()
    {
        return Selection.activeGameObject != null && Selection.activeGameObject.GetComponent<Animator>() != null;
    }

    private static List<BoneSource> BuildBoneSources(Animator animator)
    {
        var humanBoneOf = animator.isHuman ? BuildHumanBoneMap(animator) : null;
        var source = new List<BoneSource>();
        VisitTransform(animator.transform, -1, humanBoneOf, source);
        return source;
    }

    private static void VisitTransform(
        Transform current, int parentIndex,
        Dictionary<Transform, HumanBodyBones> humanBoneOf, List<BoneSource> source)
    {
        var humanBone = HumanBodyBones.LastBone;
        if (humanBoneOf != null && humanBoneOf.TryGetValue(current, out var mapped))
            humanBone = mapped;

        var index = source.Count;
        source.Add(new BoneSource
        {
            name = current.name,
            parentIndex = parentIndex,
            restLocalPosition = current.localPosition,
            restLocalRotation = current.localRotation,
            humanBone = humanBone
        });

        for (var i = 0; i < current.childCount; i++)
        {
            VisitTransform(current.GetChild(i), index, humanBoneOf, source);
        }
    }

    private static Dictionary<Transform, HumanBodyBones> BuildHumanBoneMap(Animator animator)
    {
        var map = new Dictionary<Transform, HumanBodyBones>();
        for (var bone = HumanBodyBones.Hips; bone < HumanBodyBones.LastBone; bone++)
        {
            var boneTransform = animator.GetBoneTransform(bone);
            if (boneTransform != null) map[boneTransform] = bone;
        }

        return map;
    }
}
}
