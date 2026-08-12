using Unity.Collections;
using Unity.Mathematics;

namespace AnimationTools
{
/// <summary>
/// Unmanaged, Burst-compatible mirror of a <see cref="SkeletonAsset"/>'s bone hierarchy.
/// Arrays are indexed in the same depth-first order as the source asset, so
/// <c>ParentIndices[i] &lt; i</c> for every bone and index 0 is always the root.
/// Obtained via <see cref="SkeletonAsset.GetSkeletonData"/>; do not Dispose it, the
/// asset owns the backing arrays.
/// </summary>
public struct SkeletonData
{
    public NativeArray<int> ParentIndices;
    public NativeArray<float3> RestLocalPositions;
    public NativeArray<quaternion> RestLocalRotations;
    public int BoneCount;

    public bool IsCreated => ParentIndices.IsCreated;
}
}
