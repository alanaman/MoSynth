using System;
using UnityEngine;

namespace AnimationTools
{
/// <summary>
/// Bone-lookup helpers over <see cref="SkeletonAsset"/> preserving the semantics of the
/// legacy MotionMatching skeleton lookups.
/// </summary>
public static class SkeletonAssetExtensions
{
    /// <summary>
    /// Index of the bone mapped to <paramref name="type"/>; throws if the skeleton has none.
    /// </summary>
    public static int RequireJointIndex(this SkeletonAsset asset, HumanBodyBones type)
    {
        if (asset.TryFindByHumanBone(type, out var index)) return index;
        throw new Exception($"Joint of type {type} not found in skeleton.");
    }

    /// <summary>
    /// Index of the bone mapped to <paramref name="type"/>, or 0 (the root) with an assert
    /// when missing — matching the legacy TryFind behavior of yielding a default joint
    /// (index 0) on failure.
    /// </summary>
    public static int FindJointIndexOrZero(this SkeletonAsset asset, HumanBodyBones type)
    {
        if (asset.TryFindByHumanBone(type, out var index)) return index;
        Debug.Assert(false, $"The skeleton does not contain any joint of type {type}");
        return 0;
    }
}
}
