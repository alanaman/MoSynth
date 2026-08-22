using System;
using AnimationTools;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
[Serializable]
public class PoseSetVisualizerStage : MoSynthStage
{
    private MotionSynthesisComponent _owner;
    public MotionMatchingData mmData;
    private PoseSet _poseSet;
    
    public int CurrentFrame { get; private set; }
    private float _currentFrameTime;
    
    [SerializeField]
    private int startFrame;
    
    
    public override Skeleton GetSkeleton(Skeleton inSkeleton)
    {
        _poseSet = mmData.GetOrImportPoseSet();
        return _poseSet.Skeleton;
    }
    
    public override void Init(MotionSynthesisComponent motionSynthesisComponent)
    {
        _owner = motionSynthesisComponent;
        _poseSet = mmData.GetOrImportPoseSet();
        CurrentFrame = startFrame;
    }

    public override bool Apply(PoseBuffer pose, float deltaTime)
    {
        // Advance frames with time
        _currentFrameTime = CurrentFrame + math.frac(_currentFrameTime);
        _currentFrameTime += deltaTime / _poseSet.FrameTime;
        CurrentFrame = (int)math.floor(_currentFrameTime);

        if (CurrentFrame >= _poseSet.NumberPoses)
        {
            CurrentFrame = 0;
        }
        pose.CopyFrom(_poseSet.GetPoseBuffer(CurrentFrame));
        return true;
    }
}
}