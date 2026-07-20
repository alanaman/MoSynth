
using System;
using UnityEngine;

namespace MotionMatching
{
[Serializable]
public class RetargetingStage : MoSynthStage
{
    Quaternion[] _characterTPose;
    Quaternion[] _templateTPose;
    
    public MotionMatchingData mmData;
    
    public override void Init(MotionSynthesisComponent motionSynthesisComponent)
    {
        var bodyJoints = MotionMatchingSkinnedMeshRenderer.BodyJoints;
        
        _characterTPose = new Quaternion[bodyJoints.Length];
        _templateTPose = new Quaternion[bodyJoints.Length];
        
        var tPoseAnimation = mmData.AnimationDataTPose.GetAnimation();
        var skeleton = tPoseAnimation.Skeleton;
        for (int i = 0; i < bodyJoints.Length; i++)
        {
            if (mmData.GetJointName(bodyJoints[i], out string jointName) &&
                skeleton.TryFind(jointName, out Skeleton.Joint joint))
            {
                // Get the rotation for the first frame of the animation
                _characterTPose[i] = tPoseAnimation.GetWorldRotation(joint, 0);
            }
        }
        
        var templateSkeleton = _animator.avatar.humanDescription.skeleton;
        


    }
}
}