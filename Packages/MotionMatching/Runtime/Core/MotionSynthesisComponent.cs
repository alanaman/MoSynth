using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
public class MotionSynthesisComponent : MonoBehaviour
{
    public PoseVector CurrentPose;
    
    Skeleton skeleton;
    public Transform[] skeletonTransforms;
    
    [SerializeReference] [SubclassSelector] public List<MoSynthStage> stages;
    
    private void Awake()
    {
        skeleton = null;
        foreach (var stage in stages)
        {
            stage.Init(this);
            skeleton = stage.GetSkeleton(skeleton);
        }

        InitCurrentPose();
    }
    
    private void Update()
    {
        ConstructCurrentPoseFromSkeletonTransforms();
        
        var pose = new PoseVector(CurrentPose);
        foreach (var stage in stages)
        {
            stage.Apply(pose, Time.deltaTime);
        }
        
        ApplyPoseToSkeletonTransforms(pose);
    }

    void InitCurrentPose()
    {
        var numJoints = skeleton.Joints.Count + 1;
        CurrentPose = new PoseVector(numJoints);
        
        for (var i = 0; i < skeletonTransforms.Length; i++)
        {
            CurrentPose.JointLocalRotations[i] = skeletonTransforms[i].localRotation;
            CurrentPose.JointLocalPositions[i] = skeletonTransforms[i].localPosition;
            CurrentPose.JointLocalVelocities[i] = float3.zero;
            CurrentPose.JointLocalAngularVelocities[i] = float3.zero;
        }

        CurrentPose.LeftFootContact = false;
        CurrentPose.RightFootContact = false;
    }

    void ConstructCurrentPoseFromSkeletonTransforms()
    {
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
        
        for (int i = 0; i < CurrentPose.JointLocalAngularVelocities.Length; i++)
        {
            CurrentPose.JointLocalAngularVelocities[i] = math.Euler(math.mul(skeletonTransforms[0].localRotation, CurrentPose.JointLocalRotations[i]));
            CurrentPose.JointLocalVelocities[i] = (float3)skeletonTransforms[0].localPosition - CurrentPose.JointLocalPositions[i];
        }
        
        CurrentPose.JointLocalPositions[0] = skeletonTransforms[0].localPosition;
        CurrentPose.JointLocalRotations[0] = skeletonTransforms[0].localRotation;
        
        for (var i = 1; i < skeletonTransforms.Length; i++)
        {
            CurrentPose.JointLocalRotations[i] = skeletonTransforms[i].localRotation;
        }

        // hip
        CurrentPose.JointLocalPositions[1] = skeletonTransforms[1].localPosition;

        
        // CurrentPose.LeftFootContact = ?;
        // CurrentPose.RightFootContact = ?;
    }
    
    private void ApplyPoseToSkeletonTransforms(PoseVector pose)
    {
        // Motion
        if (rootPositionsMask)
        {
            // Motion Matching Root Motion + Floor Height
            Vector3 simulationBone = pose.JointLocalPositions[0];
            // simulationBone.y = _floorHeight;
            transform.position = simulationBone;
        }

        // motionMatching.SetPosAdjustment(transform.position - motionMatching.transform.position);

        // Retargeting
        for (int i = 0; i < skeleton.Joints.Count; i++)
        {
            // Motion Matching Target Rotation
            Quaternion sourceTPoseRotation = _sourceTPose[i];
            Quaternion targetTPoseRotation = _targetTPose[i];
            var newRot = pose.GetWorldSpaceRotation(skeleton, skeleton.Joints[i]);
            
            // targetTPoseRotation -> Local Target -> World (Target TPose)
            /*
                R_t = Rotation transforming from target local space to world space.
                R_s = Rotation transforming from source local space to world space.
                R_t = R_s * R_st (R_st is a matrix transforming from target local to source local space).
                // It makes sense because R_st will be mapping from target to source, and R_s from source to world.
                // The result is transforming from T to world, which is what R_t does.
                RTPose_t = RTPose_s * R_st
                R_st = (RTPose_s)^-1 * RTPose_t
                R_t = R_s * (R_st)^-1 * RTPose_t
            */
            // HipsCorrection -> World (Target TPose) -> World (Source TPose)
            // sourceTPoseRotation^-1 -> World (SourceTPose) -> Local Source
            // sourceRotation -> Local Source -> World (Source)

            skeletonTransforms[i].rotation =
                sourceRotation * Quaternion.Inverse(sourceTPoseRotation) * _hipsCorrection *
                targetTPoseRotation;


            // if (blendPoses && _previousJointMask[i] != currentJointMask)
            // {
            //     // Pose Transition
            //     float3 offsetAngVel = float3.zero;
            //     Inertialization.InertializeJointTransition(_previousJointRotations[i], float3.zero,
            //         targetRotation, float3.zero,
            //         ref _offsetJointRotations[i], ref offsetAngVel);
            // }

            // if (blendPoses)
            // {
            //     float3 offsetAngVel = float3.zero;
            //     Inertialization.InertializeJointUpdate(targetRotation, float3.zero,
            //         blendHalfLife, Time.deltaTime,
            //         ref _offsetJointRotations[i], ref offsetAngVel,
            //         out quaternion inertializedRotation, out _);
            //     _targetBones[i].rotation = inertializedRotation;
            // }
            // else
            // {
            //     _targetBones[i].rotation = targetRotation;
            // }
        }

        // Hips
        float3 targetHipsPosition = _targetBones[0].position;
        if (rootPositionsMask)
        {
            targetHipsPosition = motionMatching.SkeletonTransforms[1].position;
        }

        if (blendPoses && _previousHipsPositionMask != rootPositionsMask)
        {
            // Position Transition
            float3 offsetAngVel = float3.zero;
            Inertialization.InertializeJointTransition(_previousHipsPosition, float3.zero,
                targetHipsPosition, float3.zero,
                ref _offsetHipsPosition, ref offsetAngVel);
        }

        if (blendPoses)
        {
            float3 offsetAngVel = float3.zero;
            Inertialization.InertializeJointUpdate(targetHipsPosition, float3.zero,
                blendHalfLife, Time.deltaTime,
                ref _offsetHipsPosition, ref offsetAngVel,
                out float3 inertializedHipsPosition, out _);
            _targetBones[0].position = inertializedHipsPosition;
        }
        else
        {
            _targetBones[0].position = targetHipsPosition;
        }

        // Toes-Floor Penetration
        if (avoidToesFloorPenetration)
        {
            const int leftToesIndex = 17;
            const int rightToesIndex = 21;
            float soleHeightOffset = Mathf.Min(_targetBones[leftToesIndex].TransformPoint(toesSoleOffset).y,
                _targetBones[rightToesIndex].TransformPoint(toesSoleOffset).y);
            soleHeightOffset = soleHeightOffset < _floorHeight ? -soleHeightOffset : 0.0f;

            const float movingAverageFactor = 0.99f;
            _toesPenetrationMovingCorrection = _toesPenetrationMovingCorrection * movingAverageFactor +
                                               (soleHeightOffset + _floorHeight) * (1.0f - movingAverageFactor);

            Vector3 hipsPos = _targetBones[0].position;
            hipsPos.y += _toesPenetrationMovingCorrection;
            _targetBones[0].position = hipsPos;
        }

        // Update State
        UpdatePreviousInertialization();
    }

}
}