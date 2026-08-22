using System;
using UnityEngine;

namespace AnimationTools
{
/// <summary>
/// Marks a Transform as a specific skeleton bone. The single bone-reference type used both by
/// scene components and by ScriptableObjects that reference Transforms on an imported-rig
/// asset. <see cref="BoneName"/> is cached alongside the Transform reference (kept in sync by
/// the property drawer) so runtime index resolution against a <see cref="Skeleton"/> never needs
/// the rig objects to be present in a build.
/// </summary>
[Serializable]
public sealed class BoneTransform
{
    [SerializeField] private Transform bone;
    [SerializeField] private string boneName;

    public Transform Transform => bone;

    public string BoneName => !string.IsNullOrEmpty(boneName) ? boneName : (bone != null ? bone.name : null);

    public bool IsSet => bone != null || !string.IsNullOrEmpty(boneName);

    public BoneTransform()
    {
    }

    public BoneTransform(string boneName)
    {
        this.boneName = boneName;
    }

    public BoneTransform(Transform bone)
    {
        this.bone = bone;
        boneName = bone != null ? bone.name : null;
    }

    /// <summary>
    /// Resolves <see cref="BoneName"/> against a skeleton. Returns -1 when the skeleton is null,
    /// this reference is unset, or no bone with that name exists.
    /// </summary>
    public int ResolveIndex(Skeleton skeleton)
    {
        if (skeleton == null || !IsSet) return -1;
        return skeleton.TryFindByName(BoneName, out var index) ? index : -1;
    }
}
}
