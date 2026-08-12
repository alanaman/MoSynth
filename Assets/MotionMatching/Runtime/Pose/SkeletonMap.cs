using AnimationTools;
using UnityEngine;

namespace MotionMatching
{
/// <summary>
/// A per-joint index correspondence between an MM <see cref="Skeleton"/> and an
/// AnimationTools <see cref="SkeletonAsset"/>, letting adapters translate between the two
/// pose representations without either side knowing about the other's indexing.
/// </summary>
public sealed class SkeletonMap
{
    public int[] MmToAsset;
    public int[] AssetToMm;
    public bool IsIdentity;

    /// <summary>
    /// Matches every MM joint to an asset bone by name, falling back to HumanBodyBones type
    /// when the joint has one. Returns null if any MM joint can't be matched.
    /// </summary>
    public static SkeletonMap Build(Skeleton mmSkeleton, SkeletonAsset asset)
    {
        if (mmSkeleton == null || asset == null) return null;

        var joints = mmSkeleton.Joints;
        var mmToAsset = new int[joints.Count];

        for (var i = 0; i < joints.Count; i++)
        {
            var joint = joints[i];

            if (asset.TryFindByName(joint.name, out var assetIndex))
            {
                mmToAsset[i] = assetIndex;
                continue;
            }

            if (joint.type != HumanBodyBones.LastBone && asset.TryFindByHumanBone(joint.type, out assetIndex))
            {
                mmToAsset[i] = assetIndex;
                continue;
            }

            return null;
        }

        var assetToMm = new int[asset.BoneCount];
        for (var i = 0; i < assetToMm.Length; i++) assetToMm[i] = -1;
        for (var i = 0; i < mmToAsset.Length; i++) assetToMm[mmToAsset[i]] = i;

        var isIdentity = mmToAsset.Length == assetToMm.Length;
        if (isIdentity)
        {
            for (var i = 0; i < mmToAsset.Length; i++)
            {
                if (mmToAsset[i] != i)
                {
                    isIdentity = false;
                    break;
                }
            }
        }

        return new SkeletonMap { MmToAsset = mmToAsset, AssetToMm = assetToMm, IsIdentity = isIdentity };
    }
}
}
