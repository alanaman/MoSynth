namespace AnimationTools
{
/// <summary>
/// Implemented by components that own the skeleton binding for their serialized object. The
/// <see cref="BoneReference"/> drawer falls back to this when no <see cref="BoneFromAttribute"/>
/// sibling resolves.
/// </summary>
public interface ISkeletonProvider
{
    SkeletonAsset PoseSkeleton { get; }
}
}
