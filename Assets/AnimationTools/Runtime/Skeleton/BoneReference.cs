using System;
using UnityEngine;

namespace AnimationTools
{
/// <summary>
/// Serializable reference to a bone by its stable id rather than its (unstable) index,
/// so it survives skeleton edits that reorder or insert bones.
/// </summary>
[Serializable]
public struct BoneReference
{
    [SerializeField] private int boneId;

    /// <summary>
    /// Display/repair hint only, e.g. for showing "Missing: LeftHand" in the inspector
    /// when the id can no longer be resolved. Never used to resolve the bone at runtime.
    /// </summary>
    [SerializeField] private string cachedName;

    public bool IsSet => boneId > 0;
    public int BoneId => boneId;
    public string CachedName => cachedName;

    public BoneReference(int boneId, string cachedName)
    {
        this.boneId = boneId;
        this.cachedName = cachedName;
    }

    /// <summary>
    /// Resolves this reference against a skeleton. Returns -1 when unset, when
    /// <paramref name="skeleton"/> is null, or when the id no longer exists in it.
    /// </summary>
    public int ResolveIndex(SkeletonAsset skeleton)
    {
        if (!IsSet || skeleton == null) return -1;
        return skeleton.IndexOfId(boneId);
    }
}
}
