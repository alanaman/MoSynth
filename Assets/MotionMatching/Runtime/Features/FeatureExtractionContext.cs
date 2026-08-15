using AnimationTools;
using Unity.Mathematics;

namespace MotionMatching
{
/// <summary>
/// The poses and character frame one feature vector is extracted from. Features are expressed
/// relative to the simulation bone, so every accessor here returns character-local values.
/// </summary>
public readonly struct FeatureExtractionContext
{
    private readonly PoseSet _poseSet;
    private readonly MotionMatchingData _mmData;
    private readonly SkeletonData _skeleton;
    private readonly int _poseIndex;
    private readonly PoseBuffer _nextPose;

    public PoseBuffer CurrentPose { get; }

    public FeatureExtractionContext(PoseSet poseSet, MotionMatchingData mmData, int poseIndex, int nextPoseIndex)
    {
        _poseSet = poseSet;
        _mmData = mmData;
        _skeleton = poseSet.SkeletonAsset.GetSkeletonData();
        _poseIndex = poseIndex;
        _nextPose = poseSet.GetPoseBuffer(nextPoseIndex);

        CurrentPose = poseSet.GetPoseBuffer(poseIndex);
    }

    public PoseBuffer GetFuturePose(int framesAhead)
    {
        return _poseSet.GetPoseBuffer(_poseIndex + framesAhead);
    }

    public float3 JointPositionLocal(PoseBuffer pose, int boneIndex)
    {
        var origin = CurrentPose.Positions[0];
        var forward = math.mul(CurrentPose.Rotations[0], math.forward());
        var worldPosition = PoseBufferFK.WorldPosition(_skeleton, pose, boneIndex);
        return FeatureSet.GetLocalPositionFromCharacter(worldPosition, origin, forward);
    }

    public float3 JointVelocityLocal(int boneIndex)
    {
        var localPosition = JointPositionLocal(CurrentPose, boneIndex);
        var localPositionNext = JointPositionLocal(_nextPose, boneIndex);
        return (localPositionNext - localPosition) / _poseSet.FrameTime;
    }

    public quaternion JointWorldRotation(PoseBuffer pose, int boneIndex)
    {
        return PoseBufferFK.WorldRotation(_skeleton, pose, boneIndex);
    }

    public float3 DirectionToLocal(float3 worldDirection)
    {
        var forward = math.mul(CurrentPose.Rotations[0], math.forward());
        return FeatureSet.GetLocalDirectionFromCharacter(worldDirection, forward);
    }

    /// <summary>Forward vector of a joint in its own local space, taken from the T-Pose.</summary>
    public float3 LocalForward(int boneIndex)
    {
        return _mmData.GetLocalForward(boneIndex);
    }
}
}
