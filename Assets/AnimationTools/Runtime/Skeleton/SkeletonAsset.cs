using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace AnimationTools
{
/// <summary>
/// A single bone in a <see cref="SkeletonAsset"/>'s depth-first bone list.
/// </summary>
[Serializable]
public struct Bone
{
    /// <summary>Stable, unique within the asset, greater than 0, never reused even after deletion.</summary>
    public int id;

    public string name;

    /// <summary>Index into the owning asset's bone list; -1 for the root.</summary>
    public int parentIndex;

    public float3 restLocalPosition;
    public quaternion restLocalRotation;

    /// <summary><see cref="HumanBodyBones.LastBone"/> when this bone has no Mecanim mapping.</summary>
    public HumanBodyBones humanBone;
}

/// <summary>
/// Authoring-time skeleton definition: a depth-first bone hierarchy with rest pose and
/// optional Mecanim mapping. Bones are referenced elsewhere by stable id (see
/// <see cref="BoneReference"/>), not by index, so the hierarchy can be edited without
/// invalidating existing references.
/// </summary>
[CreateAssetMenu(menuName = "AnimationTools/Skeleton", fileName = "NewSkeleton")]
public sealed class SkeletonAsset : ScriptableObject
{
    // Depth-first order; invariant: bones[i].parentIndex < i for all i, and bones[0] is the
    // sole root (parentIndex == -1).
    [SerializeField] private List<Bone> bones = new();

    // Monotonic id allocator. Bone ids start at 1; id 0 means "unset" so a default-constructed
    // BoneReference resolves to nothing.
    [SerializeField] private int nextBoneId = 1;

    // Bumped on every structural change (SetBones). Consumers cache derived data
    // (layouts, SkeletonData) keyed on this and rebuild when it changes.
    [SerializeField] private int version;

    [NonSerialized] private Dictionary<int, int> idToIndex;
    [NonSerialized] private SkeletonData cachedSkeletonData;
    [NonSerialized] private int cachedSkeletonDataVersion = -1;

    public int BoneCount => bones.Count;
    public int Version => version;
    public int NextBoneId => nextBoneId;

    public Bone GetBone(int index) => bones[index];

    /// <summary>Returns -1 if no bone with this id exists.</summary>
    public int IndexOfId(int boneId)
    {
        EnsureIdMap();
        return idToIndex.GetValueOrDefault(boneId, -1);
    }

    public bool TryFindByHumanBone(HumanBodyBones humanBone, out int index)
    {
        for (var i = 0; i < bones.Count; i++)
        {
            if (bones[i].humanBone != humanBone) continue;
            index = i;
            return true;
        }

        index = -1;
        return false;
    }

    public bool TryFindByName(string boneName, out int index)
    {
        for (var i = 0; i < bones.Count; i++)
        {
            if (bones[i].name != boneName) continue;
            index = i;
            return true;
        }

        index = -1;
        return false;
    }

    /// <summary>
    /// Mutation entry point for editor authoring code. Validates <paramref name="newBones"/>
    /// (depth-first parent invariant, unique positive ids, single root at index 0,
    /// <paramref name="newNextBoneId"/> greater than every id present) before committing,
    /// then bumps <see cref="Version"/> and invalidates caches.
    /// </summary>
    public void SetBones(List<Bone> newBones, int newNextBoneId)
    {
        if (newBones == null || newBones.Count == 0)
            throw new ArgumentException("Skeleton must contain at least one bone.", nameof(newBones));

        if (newBones[0].parentIndex != -1)
            throw new ArgumentException("Bone at index 0 must be the root (parentIndex == -1).", nameof(newBones));

        var seenIds = new HashSet<int>();
        for (var i = 0; i < newBones.Count; i++)
        {
            var bone = newBones[i];

            if (bone.id <= 0)
                throw new ArgumentException($"Bone \"{bone.name}\" at index {i} has non-positive id {bone.id}.",
                    nameof(newBones));

            if (!seenIds.Add(bone.id))
                throw new ArgumentException($"Duplicate bone id {bone.id} at index {i}.", nameof(newBones));

            if (i == 0) continue;

            if (bone.parentIndex < 0)
                throw new ArgumentException($"Bone \"{bone.name}\" at index {i} has no parent but is not the root.",
                    nameof(newBones));

            if (bone.parentIndex >= i)
                throw new ArgumentException(
                    $"Bone \"{bone.name}\" at index {i} has parentIndex {bone.parentIndex}, which violates the depth-first invariant (parentIndex must be < index).",
                    nameof(newBones));
        }

        var maxId = 0;
        foreach (var id in seenIds) maxId = math.max(maxId, id);
        if (newNextBoneId <= maxId)
            throw new ArgumentException(
                $"newNextBoneId ({newNextBoneId}) must be greater than every existing bone id (max {maxId}).",
                nameof(newNextBoneId));

        bones = new List<Bone>(newBones);
        nextBoneId = newNextBoneId;
        version++;
        idToIndex = null;
    }

    /// <summary>
    /// Returns the unmanaged mirror of this skeleton, rebuilt only when <see cref="Version"/>
    /// changes since the last call. Backing arrays use <see cref="Allocator.Domain"/>: they
    /// live for the domain's lifetime and are cleared automatically on domain reload, so
    /// callers must not Dispose them.
    /// </summary>
    public SkeletonData GetSkeletonData()
    {
        if (cachedSkeletonDataVersion == version && cachedSkeletonData.IsCreated) return cachedSkeletonData;

        var count = bones.Count;
        var parentIndices = new NativeArray<int>(count, Allocator.Domain);
        var restLocalPositions = new NativeArray<float3>(count, Allocator.Domain);
        var restLocalRotations = new NativeArray<quaternion>(count, Allocator.Domain);

        for (var i = 0; i < count; i++)
        {
            var bone = bones[i];
            parentIndices[i] = bone.parentIndex;
            restLocalPositions[i] = bone.restLocalPosition;
            restLocalRotations[i] = bone.restLocalRotation;
        }

        cachedSkeletonData = new SkeletonData
        {
            ParentIndices = parentIndices,
            RestLocalPositions = restLocalPositions,
            RestLocalRotations = restLocalRotations,
            BoneCount = count
        };
        cachedSkeletonDataVersion = version;

        return cachedSkeletonData;
    }

    private void EnsureIdMap()
    {
        if (idToIndex != null) return;

        idToIndex = new Dictionary<int, int>(bones.Count);
        for (var i = 0; i < bones.Count; i++)
        {
            idToIndex[bones[i].id] = i;
        }
    }

    private void OnValidate()
    {
        idToIndex = null;
    }
}
}