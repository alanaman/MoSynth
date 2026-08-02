using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
public class MotionSynthesisComponent : MotionSynthesizer
{
    // TODO: remove. shouldn't be coupled to MotionMatchingData
    public MotionMatchingData mmData;
    
    [NonSerialized]
    public PoseVector CurrentPose;

    private Skeleton _skeleton;
    public Skeleton Skeleton => _skeleton;
    
    /// <summary>
    /// The transforms of the character controlled by this <see cref="MotionSynthesisComponent"/>.
    /// These are the transforms that will be used to render the character.
    /// </summary>
    [NonSerialized]
    public Transform[] SkeletonTransforms;
    
    // TODO: remove?
    private Animator _animator;
    
    [SerializeField]
    [Tooltip("The avatar that maps from mecanim template to character transforms.")]
    private Avatar avatar;
    
    [SerializeReference] [SubclassSelector]
    public List<MoSynthStage> stages = new();

    [Tooltip("Whether to animate the root position by Motion Matching or not.")]
    // maybe change this to 'root motion'?
    public bool rootPositionsMask = true;
    
    public override MotionMatchingData MmData => mmData;
    public override float3 RootVelocity { get; protected set; }
    public override float3 RootAngularVelocity { get; protected set; }
    public override float3 RootPosition { get; protected set; }
    public override quaternion RootRotation { get; protected set; }
    
    private void Awake()
    {
        _animator = GetComponent<Animator>();
        
        _skeleton = null;
        
        stages.RemoveAll(stage => stage == null);
        foreach (var stage in stages)
        {
            _skeleton = stage.GetSkeleton(_skeleton);
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
        var poseSet = mmData.GetOrImportPoseSet();
        var poseJoints = poseSet.Skeleton.Joints;
        
        // +1 for SimulationBone
        SkeletonTransforms = new Transform[poseJoints.Count];
        SkeletonTransforms[0] = transform; // SimulationBone
        for (var i = 1; i < poseJoints.Count; i++)
        {
            SkeletonTransforms[i] = _animator.GetBoneTransform(poseJoints[i].type);
        }
    }
    
    private void ApplyTransformOffsetsFromSkeleton()
    {
        var poseSet = mmData.GetOrImportPoseSet();
        var animJoints = poseSet.Skeleton.Joints;
        
        for (var i = 1; i < animJoints.Count; i++)
        {
            var skeletonTransform = _animator.GetBoneTransform(animJoints[i].type);
            var skeletonBone = GetSkeletonBone(_animator.avatar.humanDescription, animJoints[i].type);
            skeletonTransform.localPosition = skeletonBone.position;
        }
    }
    
    /// <summary>
    /// Finds the <see cref="SkeletonBone"/> in <paramref name="humanDescription"/>, corresponding to the <see cref="HumanBodyBones"/> <paramref name="boneEnum"/>.
    /// </summary>
    /// <returns>The <see cref="SkeletonBone"/> corresponding to <paramref name="boneEnum"/></returns>
    public SkeletonBone GetSkeletonBone(HumanDescription humanDescription, HumanBodyBones boneEnum)
    {
        // Ensure we aren't passing an invalid enum value
        if (boneEnum < 0 || boneEnum >= HumanBodyBones.LastBone)
        {
            throw new ArgumentException("Invalid HumanBodyBones enum value.");
        }

        // Get the standard Mecanim human bone name (e.g., "LeftUpperArm")
        var targetHumanName = HumanTrait.BoneName[(int)boneEnum];

        // Find the mapped transform name in the rig
        var targetRigBoneName = humanDescription.human.First(b => b.humanName == targetHumanName).boneName;
        
        
        return humanDescription.skeleton.First(skeletonBone => skeletonBone.name == targetRigBoneName);
    }

    private void LateUpdate()
    {
        ConstructCurrentPoseFromSkeletonTransforms();
        
        var pose = new PoseVector(CurrentPose);
        foreach (var stage in stages)
        {
            if (stage.isEnabled)
            {
                if(!stage.Apply(pose, Time.deltaTime))
                {
                    break;
                }
            }
        }
        
        ApplyPoseToSkeletonTransforms(pose);
    }

    void InitCurrentPose()
    {
        CurrentPose = new PoseVector(_skeleton.Joints.Count);
        
        for (var i = 0; i < SkeletonTransforms.Length; i++)
        {
            CurrentPose.jointLocalRotations[i] = SkeletonTransforms[i].localRotation;
            CurrentPose.jointLocalPositions[i] = SkeletonTransforms[i].localPosition;
            CurrentPose.jointLocalVelocities[i] = float3.zero;
            CurrentPose.jointLocalAngularVelocities[i] = float3.zero;
        }

        CurrentPose.leftFootContact = false;
        CurrentPose.rightFootContact = false;
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
        
        for (var i = 0; i < CurrentPose.jointLocalAngularVelocities.Length; i++)
        {
            var inverseLocalRotation = Quaternion.Inverse(CurrentPose.jointLocalRotations[i]);
            CurrentPose.jointLocalAngularVelocities[i] = (SkeletonTransforms[i].localRotation * inverseLocalRotation).eulerAngles;
            CurrentPose.jointLocalVelocities[i] = (float3)SkeletonTransforms[i].localPosition - CurrentPose.jointLocalPositions[i];
        }
        
        CurrentPose.jointLocalPositions[0] = SkeletonTransforms[0].localPosition;
        CurrentPose.jointLocalRotations[0] = SkeletonTransforms[0].localRotation;
        
        for (var i = 1; i < SkeletonTransforms.Length; i++)
        {
            CurrentPose.jointLocalRotations[i] = SkeletonTransforms[i].localRotation;
        }

        // hip
        CurrentPose.jointLocalPositions[1] = SkeletonTransforms[1].localPosition;

        
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

        for (var i = 1; i < _skeleton.Joints.Count; i++)
        {
            SkeletonTransforms[i].localRotation = pose.jointLocalRotations[i];
        }

        // hips
        SkeletonTransforms[1].localPosition = pose.jointLocalPositions[1];
        
        // root
        if (rootPositionsMask)
        {
            var rootTransform = SkeletonTransforms[0];
            rootTransform.localPosition += rootTransform.localRotation * pose.jointLocalVelocities[0] * Time.deltaTime;
            var angVel = pose.jointLocalAngularVelocities[0];
            var rootRotation = quaternion.AxisAngle(angVel, math.length(angVel) * Time.deltaTime);
            rootTransform.localRotation *= rootRotation;
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

    private void OnDestroy()
    {
        foreach (var stage in stages)
        {
            stage?.OnDestroy();
        }
    }


    public override void SetRotAdjustment(quaternion adjustmentRotation)
    {
        throw new NotImplementedException();
    }

    public override void SetPosAdjustment(float3 adjustmentPosition)
    {
        throw new NotImplementedException();
    }

    public override float3 GetMainPositionFeature(int trajectoryIndex)
    {
        throw new NotImplementedException();
    }

    public override float4 GetEnvironmentFeature(string featureName, int trajectoryIndex)
    {
        throw new NotImplementedException();
    }
}
}