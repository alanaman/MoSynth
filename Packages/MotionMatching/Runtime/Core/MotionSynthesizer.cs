using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Serialization;

namespace MotionMatching
{
public class MotionSynthesizer : MonoBehaviour
{
    public PoseVector CurrentPose;
    
    public Transform[] skeletonTransforms;
    
    PoseVector ConstructPoseFromSkeletonTransforms(in PoseVector prevPose, Transform[] skeletonTransforms)
    {
        var pose = new PoseVector();
        
        // // Simulation Bone
        // float3 pos = skeletonTransforms[0].position;
        // var rot = skeletonTransforms[0].rotation;
        //
        // // world space to local space
        // var localSpacePos = math.mul(math.inverse(_mmTransformOriginRot), (pos - _mmTransformOriginPos));
        // var localSpaceRot = math.mul(math.inverse(_mmTransformOriginRot), rot);
        //
        // // local space to animation space
        // pose.JointLocalRotations[0] = math.mul(_animationSpaceOriginRot, localSpaceRot);
        //
        // pose.JointLocalPositions[0] = math.mul(_inverseAnimationSpaceOriginRot, localSpacePos) + _animationSpaceOriginPos;
        
        pose.JointLocalPositions[0] = skeletonTransforms[0].localPosition;
        pose.JointLocalRotations[0] = skeletonTransforms[0].localRotation;
        
        for (var i = 1; i < skeletonTransforms.Length; i++)
        {
            pose.JointLocalRotations[i] = skeletonTransforms[i].localRotation;
        }

        // hip
        pose.JointLocalPositions[1] = skeletonTransforms[1].localPosition;

        for (int i = 0; i < pose.JointLocalAngularVelocities.Length; i++)
        {
            pose.JointLocalAngularVelocities[i] = math.Euler(math.mul(pose.JointLocalRotations[i], prevPose.JointLocalRotations[i]));
            pose.JointLocalVelocities[i] = pose.JointLocalPositions[i] - prevPose.JointLocalPositions[i];
        }
        
        pose.LeftFootContact = ?;
        pose.RightFootContact = ?;
                
        return pose;
    }
}
}