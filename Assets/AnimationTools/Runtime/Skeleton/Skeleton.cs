using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;

namespace AnimationTools
{
/// <summary>
/// A single bone in a <see cref="AnimationTools.Skeleton"/>'s depth-first bone list.
/// </summary>
[Serializable]
public struct SkeletonBoneData
{
    public string name;

    /// <summary>Index into the owning skeleton's bone list; -1 for the root (index 0 only).</summary>
    public int parentIndex;

    public float3 restLocalPosition;
    public quaternion restLocalRotation;
}

/// <summary>
/// Immutable, plain-C# skeleton definition: a depth-first bone hierarchy with rest pose. Not a
/// <see cref="UnityEngine.Object"/>; bones are referenced by index, which is stable for the
/// lifetime of the instance because the instance itself can never be mutated after construction.
/// </summary>
public sealed class Skeleton
{
    private readonly List<SkeletonBoneData> _bones;

    private SkeletonData _cachedSkeletonData;
    private bool _hasCachedSkeletonData;

    /// <summary>Diagnostic label; not used for lookups or equality.</summary>
    public string Name { get; }

    public int BoneCount => _bones.Count;

    /// <summary>
    /// Hash over each bone's (name, parentIndex), computed once at construction. Two skeletons
    /// with the same <see cref="ContentHash"/> are very likely (though not guaranteed)
    /// structurally identical; use <see cref="StructurallyEqual"/> to confirm.
    /// </summary>
    public int ContentHash { get; }

    public Skeleton(IReadOnlyList<SkeletonBoneData> bones, string name = "Skeleton")
    {
        if (bones == null || bones.Count == 0)
            throw new ArgumentException("Skeleton must contain at least one bone.", nameof(bones));

        if (bones[0].parentIndex != -1)
            throw new ArgumentException("Bone at index 0 must be the root (parentIndex == -1).", nameof(bones));

        for (var i = 1; i < bones.Count; i++)
        {
            var bone = bones[i];

            if (bone.parentIndex < 0)
                throw new ArgumentException($"Bone \"{bone.name}\" at index {i} has no parent but is not the root.",
                    nameof(bones));

            if (bone.parentIndex >= i)
                throw new ArgumentException(
                    $"Bone \"{bone.name}\" at index {i} has parentIndex {bone.parentIndex}, which violates the depth-first invariant (parentIndex must be < index).",
                    nameof(bones));
        }

        _bones = new List<SkeletonBoneData>(bones);
        Name = name;
        ContentHash = ComputeContentHash(_bones);
    }

    public SkeletonBoneData GetBone(int index) => _bones[index];

    public bool TryFindByName(string boneName, out int index)
    {
        for (var i = 0; i < _bones.Count; i++)
        {
            if (_bones[i].name != boneName) continue;
            index = i;
            return true;
        }

        index = -1;
        return false;
    }

    /// <summary>Returns -1 if no bone with this name exists.</summary>
    public int IndexOfName(string boneName)
    {
        return TryFindByName(boneName, out var index) ? index : -1;
    }

    /// <summary>
    /// Returns -1 for an unset id (0) or one outside the valid range. Bone ids follow the
    /// index + 1 convention so 0 unambiguously means "unset".
    /// </summary>
    public int IndexOfId(int boneId)
    {
        if (boneId < 1 || boneId > BoneCount) return -1;
        return boneId - 1;
    }

    public int GetBoneId(int index) => index + 1;

    /// <summary>
    /// Character-space rotation of one bone in the rest pose, found by walking up its parent chain.
    /// "Character space" matches <see cref="PoseFK.CharacterRotation"/>: the frame of bone 0's parent.
    /// </summary>
    public quaternion RestCharacterRotation(int boneIndex)
    {
        var rotation = quaternion.identity;

        while (boneIndex != -1)
        {
            rotation = math.mul(_bones[boneIndex].restLocalRotation, rotation);
            boneIndex = _bones[boneIndex].parentIndex;
        }

        return rotation;
    }

    /// <summary>
    /// The bone-local axis that points along <paramref name="characterAxis"/> in the rest pose.
    /// A bone's local frame carries whatever roll the rig was authored with, so the axis that means
    /// "forward" differs per rig and must be read from the rest pose rather than assumed.
    /// </summary>
    public float3 RestLocalAxis(int boneIndex, float3 characterAxis) =>
        math.mul(math.inverse(RestCharacterRotation(boneIndex)), characterAxis);

    /// <summary>
    /// Returns the unmanaged mirror of this skeleton, built once and cached for the lifetime of
    /// this instance (which is immutable). Backing arrays use <see cref="Allocator.Domain"/>:
    /// they live for the domain's lifetime and are cleared automatically on domain reload, so
    /// callers must not Dispose them.
    /// </summary>
    public SkeletonData GetSkeletonData()
    {
        if (_hasCachedSkeletonData && _cachedSkeletonData.IsCreated) return _cachedSkeletonData;

        var count = _bones.Count;
        var parentIndices = new NativeArray<int>(count, Allocator.Domain);
        var restLocalPositions = new NativeArray<float3>(count, Allocator.Domain);
        var restLocalRotations = new NativeArray<quaternion>(count, Allocator.Domain);

        for (var i = 0; i < count; i++)
        {
            var bone = _bones[i];
            parentIndices[i] = bone.parentIndex;
            restLocalPositions[i] = bone.restLocalPosition;
            restLocalRotations[i] = bone.restLocalRotation;
        }

        _cachedSkeletonData = new SkeletonData
        {
            ParentIndices = parentIndices,
            RestLocalPositions = restLocalPositions,
            RestLocalRotations = restLocalRotations,
            BoneCount = count
        };
        _hasCachedSkeletonData = true;

        return _cachedSkeletonData;
    }

    /// <summary>
    /// Returns a new skeleton with a synthetic root bone prepended at index 0. Every bone of
    /// this skeleton shifts to index i + 1, with its parentIndex shifted by 1; the original root
    /// (old index 0) becomes a child of the new root. Mirrors the SimulationBone prepend done by
    /// <see cref="PoseSet.SetSkeletonFromBvh"/>.
    /// </summary>
    public Skeleton WithRootPrepended(string rootName)
    {
        var newBones = new List<SkeletonBoneData>(_bones.Count + 1)
        {
            new()
            {
                name = rootName,
                parentIndex = -1,
                restLocalPosition = float3.zero,
                restLocalRotation = quaternion.identity
            }
        };

        for (var i = 0; i < _bones.Count; i++)
        {
            var bone = _bones[i];
            newBones.Add(new SkeletonBoneData
            {
                name = bone.name,
                parentIndex = i == 0 ? 0 : bone.parentIndex + 1,
                restLocalPosition = bone.restLocalPosition,
                restLocalRotation = bone.restLocalRotation
            });
        }

        return new Skeleton(newBones, Name);
    }

    /// <summary>
    /// True when both skeletons have the same bone count and every bone matches by name and
    /// parentIndex. Rest pose is not compared. Null-safe: two nulls (or the same reference)
    /// are considered equal.
    /// </summary>
    public static bool StructurallyEqual(Skeleton a, Skeleton b)
    {
        if (ReferenceEquals(a, b)) return true;
        if (a == null || b == null) return false;
        if (a.BoneCount != b.BoneCount) return false;

        for (var i = 0; i < a.BoneCount; i++)
        {
            var boneA = a._bones[i];
            var boneB = b._bones[i];
            if (boneA.name != boneB.name) return false;
            if (boneA.parentIndex != boneB.parentIndex) return false;
        }

        return true;
    }

    private static int ComputeContentHash(List<SkeletonBoneData> bones)
    {
        unchecked
        {
            var hash = 17;
            foreach (var bone in bones)
            {
                hash = hash * 397 ^ (bone.name != null ? bone.name.GetHashCode() : 0);
                hash = hash * 397 ^ bone.parentIndex;
            }

            return hash;
        }
    }
}
}
