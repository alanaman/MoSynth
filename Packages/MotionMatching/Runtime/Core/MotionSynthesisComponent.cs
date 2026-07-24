using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
public class MotionSynthesisComponent : MonoBehaviour
{
    // TODO: remove. shouldn't be coupled to MotionMatchingData
    public MotionMatchingData mmData;
    
    public PoseVector CurrentPose;
    
    Skeleton skeleton;
    public Skeleton Skeleton => skeleton;
    
    /// <summary>
    /// The transforms of the character controlled by this <see cref="MotionSynthesisComponent"/>.
    /// These are the transforms that will be used to render the character.
    /// </summary>
    public Transform[] skeletonTransforms;
    
    // TODO: remove?
    private Animator _animator;
    
    [SerializeField]
    [Tooltip("The avatar that maps from mecanim template to character tranforms.")]
    private Avatar avatar;
    
    [SerializeReference] [SubclassSelector] public List<MoSynthStage> stages;

    [Tooltip("Whether to animate the root position by Motion Matching or not.")]
    // maybe change this to 'root motion'?
    public bool rootPositionsMask = true;
    
    private void Awake()
    {
        _animator = GetComponent<Animator>();
        
        skeleton = null;
        foreach (var stage in stages)
        {
            skeleton = stage.GetSkeleton(skeleton);
        }

        InitSkeletonTransformsArray();
        InitCurrentPose();
        ApplyTransformOffsetsFromSkeleton();

        foreach (var stage in stages)
        {
            stage.Init(this);
        }
    }


    private void InitSkeletonTransformsArray()
    {
        var bodyJoints = MotionMatchingSkinnedMeshRenderer.BodyJoints;
        // +1 for SimulationBone
        skeletonTransforms = new Transform[bodyJoints.Length + 1];
        skeletonTransforms[0] = transform; // SimulationBone
        for (int i = 0; i < bodyJoints.Length; i++)
        {
            skeletonTransforms[i + 1] = _animator.GetBoneTransform(bodyJoints[i]);
        }
    }
    
    private void ApplyTransformOffsetsFromSkeleton()
    {
        var poseSet = mmData.GetOrImportPoseSet();
        
        var bodyJoints = MotionMatchingSkinnedMeshRenderer.BodyJoints;
        var templateSkeleton = _animator.avatar.humanDescription.skeleton;
        for (int i = 0; i < bodyJoints.Length; i++)
        {
            var skeletonTransform = _animator.GetBoneTransform(bodyJoints[i]);
            var targetJointIndex = Array.FindIndex(templateSkeleton, bone => bone.name == skeletonTransform.name);
            var skeletonBone = templateSkeleton[targetJointIndex];
            // var skeletonJoint = poseSet.Skeleton.Joints[i+1];
            
            skeletonTransform.localPosition = skeletonBone.position;
        }
    }

    private void Update()
    {
        ConstructCurrentPoseFromSkeletonTransforms();
        
        var pose = new PoseVector(CurrentPose);
        foreach (var stage in stages)
        {
            if (stage.isEnabled)
            {
                stage.Apply(pose, Time.deltaTime);
            }
        }
        
        ApplyPoseToSkeletonTransforms(pose);
    }

    void InitCurrentPose()
    {
        CurrentPose = new PoseVector(skeleton.Joints.Count);
        
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
            var inverseLocalRotation = Quaternion.Inverse(CurrentPose.JointLocalRotations[i]);
            CurrentPose.JointLocalAngularVelocities[i] = (skeletonTransforms[i].localRotation * inverseLocalRotation).eulerAngles;
            CurrentPose.JointLocalVelocities[i] = skeletonTransforms[i].localPosition - CurrentPose.JointLocalPositions[i];
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
        // if (rootPositionsMask)
        // {
        //     // Motion Matching Root Motion + Floor Height
        //     Vector3 simulationBone = pose.JointLocalPositions[0];
        //     // simulationBone.y = _floorHeight;
        //     transform.position = simulationBone;
        // }

        var bodyJoints = MotionMatchingSkinnedMeshRenderer.BodyJoints;
        for (int i = 1; i < skeleton.Joints.Count; i++)
        {
            skeletonTransforms[i].localRotation = pose.JointLocalRotations[i];
        }

        // motionMatching.SetPosAdjustment(transform.position - motionMatching.transform.position);
        if (rootPositionsMask)
        {
            skeletonTransforms[0].localPosition = pose.JointLocalPositions[0];
        }

        // if (blendPoses && _previousHipsPositionMask != rootPositionsMask)
        // {
        //     // Position Transition
        //     float3 offsetAngVel = float3.zero;
        //     Inertialization.InertializeJointTransition(_previousHipsPosition, float3.zero,
        //         targetHipsPosition, float3.zero,
        //         ref _offsetHipsPosition, ref offsetAngVel);
        // }
        //
        // if (blendPoses)
        // {
        //     float3 offsetAngVel = float3.zero;
        //     Inertialization.InertializeJointUpdate(targetHipsPosition, float3.zero,
        //         blendHalfLife, Time.deltaTime,
        //         ref _offsetHipsPosition, ref offsetAngVel,
        //         out float3 inertializedHipsPosition, out _);
        //     _targetBones[0].position = inertializedHipsPosition;
        // }
        // else
        // {
        //     _targetBones[0].position = targetHipsPosition;
        // }

        // // Toes-Floor Penetration
        // if (avoidToesFloorPenetration)
        // {
        //     const int leftToesIndex = 17;
        //     const int rightToesIndex = 21;
        //     float soleHeightOffset = Mathf.Min(_targetBones[leftToesIndex].TransformPoint(toesSoleOffset).y,
        //         _targetBones[rightToesIndex].TransformPoint(toesSoleOffset).y);
        //     soleHeightOffset = soleHeightOffset < _floorHeight ? -soleHeightOffset : 0.0f;
        //
        //     const float movingAverageFactor = 0.99f;
        //     _toesPenetrationMovingCorrection = _toesPenetrationMovingCorrection * movingAverageFactor +
        //                                        (soleHeightOffset + _floorHeight) * (1.0f - movingAverageFactor);
        //
        //     Vector3 hipsPos = _targetBones[0].position;
        //     hipsPos.y += _toesPenetrationMovingCorrection;
        //     _targetBones[0].position = hipsPos;
        // }
        //
        // // Update State
        // UpdatePreviousInertialization();
    }

    private void OnValidate()
    {
        foreach (var stage in stages)
        {
            stage?.OnValidate();
        }
    }
}
}