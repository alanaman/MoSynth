using System;
using System.Collections.Generic;
using UnityEngine;

namespace AnimationTools
{
/// <summary>
/// Scene-side marker that a Transform is the root of a rig, and a bridge between that rig's
/// Transform hierarchy and an immutable <see cref="AnimationTools.Skeleton"/>. Bone order is
/// always the preorder depth-first walk of the hierarchy under <see cref="Root"/>, which is the
/// project-wide skeleton bone order.
/// </summary>
[Serializable]
public sealed class SkeletonRoot
{
    /// <summary>Escape hatch for a bone whose name in the skeleton doesn't match the rig's
    /// Transform name, or that should resolve to a Transform outside the DFS walk.</summary>
    [Serializable]
    public struct BoneOverride
    {
        public string skeletonBoneName;
        public Transform target;
    }

    [SerializeField] private Transform root;
    [SerializeField] private List<BoneOverride> overrides = new();

    public Transform Root => root;
    public bool IsSet => root != null;

    /// <summary>Runtime fallback for when no root was assigned in the inspector.</summary>
    public void SetRoot(Transform newRoot)
    {
        root = newRoot;
    }

    /// <summary>
    /// Returns every Transform under
    /// <paramref name="root"/> (root included) in preorder depth-first order: a transform is visited
    /// before its children, and children are visited in child-index order. This is the
    /// project-wide skeleton bone order.
    /// </summary>
    public static List<Transform> CollectTransformsDfs(Transform root)
    {
        var result = new List<Transform>();
        if (root == null) return result;

        CollectRecursive(root, result);
        return result;
    }

    private static void CollectRecursive(Transform transform, List<Transform> results)
    {
        results.Add(transform);
        for (var i = 0; i < transform.childCount; i++)
        {
            CollectRecursive(transform.GetChild(i), results);
        }
    }

    /// <summary>
    /// Builds an immutable <see cref="AnimationTools.Skeleton"/> snapshot of the current rig
    /// pose, in DFS order. Throws when <see cref="Root"/> is unset.
    /// </summary>
    public Skeleton BuildSkeleton(string name = null)
    {
        if (root == null)
            throw new InvalidOperationException("SkeletonRoot has no root Transform assigned.");

        var transforms = CollectTransformsDfs(root);

        var bones = new List<SkeletonBoneData>(transforms.Count);
        for (var i = 0; i < transforms.Count; i++)
        {
            var transform = transforms[i];
            var parentIndex = i == 0 ? -1 : transforms.IndexOf(transform.parent);
            bones.Add(new SkeletonBoneData
            {
                name = transform.name,
                parentIndex = parentIndex,
                restLocalPosition = transform.localPosition,
                restLocalRotation = transform.localRotation
            });
        }

        return new Skeleton(bones, name ?? root.name);
    }

    /// <summary>
    /// Finds the Transform for a bone by name: overrides are consulted first, then a DFS search
    /// (root included) for an exact name match. Null when no match is found in either.
    /// </summary>
    public Transform FindBone(string boneName)
    {
        foreach (var boneOverride in overrides)
        {
            if (boneOverride.target != null && boneOverride.skeletonBoneName == boneName)
                return boneOverride.target;
        }

        return root == null ? null : FindBoneRecursive(root, boneName);
    }

    private static Transform FindBoneRecursive(Transform transform, string boneName)
    {
        if (transform.name == boneName) return transform;

        for (var i = 0; i < transform.childCount; i++)
        {
            var found = FindBoneRecursive(transform.GetChild(i), boneName);
            if (found != null) return found;
        }

        return null;
    }

    /// <summary>
    /// Resolves every bone of <paramref name="skeleton"/> to a Transform in this rig by name.
    /// Slot 0 is <paramref name="indexZeroOverride"/> when given, else <see cref="Root"/>.
    /// Missing bones log an error and leave their slot null; the caller decides how severe that
    /// is. Duplicate bone names in the hierarchy that the skeleton references also log a
    /// warning, since the first DFS match silently wins.
    /// </summary>
    public Transform[] BindByName(Skeleton skeleton, Transform indexZeroOverride = null)
    {
        var result = new Transform[skeleton.BoneCount];
        if (skeleton.BoneCount == 0) return result;

        result[0] = indexZeroOverride != null ? indexZeroOverride : root;

        var transforms = CollectTransformsDfs(root);
        var nameCounts = new Dictionary<string, int>();
        foreach (var transform in transforms)
        {
            nameCounts.TryGetValue(transform.name, out var count);
            nameCounts[transform.name] = count + 1;
        }

        for (var i = 1; i < skeleton.BoneCount; i++)
        {
            var boneName = skeleton.GetBone(i).name;
            var bone = FindBone(boneName);
            if (bone == null)
            {
                Debug.LogError($"SkeletonRoot could not find bone \"{boneName}\" under root \"{(root != null ? root.name : "<none>")}\".");
                continue;
            }

            if (nameCounts.TryGetValue(boneName, out var count) && count > 1)
            {
                Debug.LogWarning($"SkeletonRoot found {count} transforms named \"{boneName}\" under root \"{(root != null ? root.name : "<none>")}\"; using the first depth-first match.");
            }

            result[i] = bone;
        }

        return result;
    }
}
}
