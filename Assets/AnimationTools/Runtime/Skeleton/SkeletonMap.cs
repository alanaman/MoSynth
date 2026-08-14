using UnityEngine;

namespace AnimationTools
{
/// <summary>
/// A per-bone index correspondence between two <see cref="SkeletonAsset"/>s (the "Mm" side
/// being the pose-set skeleton), letting adapters translate between two pose
/// representations without either side knowing about the other's indexing.
/// </summary>
public sealed class SkeletonMap
{
    public int[] MmToAsset;
    public int[] AssetToMm;
    public bool IsIdentity;

    /// <summary>
    /// Matches every bone of <paramref name="source"/> to a bone of <paramref name="target"/>
    /// by name, falling back to HumanBodyBones type when the bone has one. Returns null if
    /// any source bone can't be matched. The source side plays the "MM" role of the map.
    /// </summary>
    public static SkeletonMap Build(SkeletonAsset source, SkeletonAsset target)
    {
        if (source == null || target == null) return null;

        var mmToAsset = new int[source.BoneCount];

        for (var i = 0; i < source.BoneCount; i++)
        {
            var bone = source.GetBone(i);

            if (target.TryFindByName(bone.name, out var targetIndex))
            {
                mmToAsset[i] = targetIndex;
                continue;
            }

            if (bone.humanBone != HumanBodyBones.LastBone && target.TryFindByHumanBone(bone.humanBone, out targetIndex))
            {
                mmToAsset[i] = targetIndex;
                continue;
            }

            return null;
        }

        return FromMmToAsset(mmToAsset, target.BoneCount);
    }

    private static SkeletonMap FromMmToAsset(int[] mmToAsset, int assetBoneCount)
    {
        var assetToMm = new int[assetBoneCount];
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
