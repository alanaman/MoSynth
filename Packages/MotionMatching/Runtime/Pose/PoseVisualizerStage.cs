using System;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
[Serializable]
public class PoseVisualizerStage : MoSynthStage
{
    private MotionSynthesisComponent _owner;
    public MotionMatchingData mmData;
    private PoseSet _poseSet;
    public bool lockFPS = true;
    
    public int CurrentFrame { get; private set; }
    private float _currentFrameTime;
    
    public override Skeleton GetSkeleton(in Skeleton inSkeleton)
    {
        _poseSet = mmData.GetOrImportPoseSet();
        return _poseSet.Skeleton;
    }
    
    public override void Init(MotionSynthesisComponent motionSynthesisComponent)
    {
        _owner = motionSynthesisComponent;
        _poseSet = mmData.GetOrImportPoseSet();
        // FPS
        var databaseFrameTime = _poseSet.FrameTime;
        var databaseFrameRate = Mathf.RoundToInt(1.0f / databaseFrameTime);
        if (lockFPS)
        {
            Application.targetFrameRate = databaseFrameRate;
            Debug.Log(
                "[Motion Matching] Updated Target FPS: " +
                Application.targetFrameRate);
        }
        else
        {
            Application.targetFrameRate = -1;
            Debug.LogWarning(
                "[Motion Matching] LockFPS is not set. Motion Matching" +
                " will malfunction if the application frame rate is higher" +
                " than the animation database.");
        }

        
    }

    public override void Apply(PoseVector pose, float deltaTime)
    {
        // Advance frames with time
        _currentFrameTime = CurrentFrame + math.frac(_currentFrameTime);
        _currentFrameTime += deltaTime / _poseSet.FrameTime;
        CurrentFrame = (int)math.floor(_currentFrameTime);

        if (CurrentFrame >= _poseSet.NumberPoses)
        {
            CurrentFrame = 0;
        }
        _poseSet.GetPose(CurrentFrame, out var newPose);
        pose.CopyFrom(newPose);
    }
}
}