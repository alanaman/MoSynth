using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace AnimationTools
{
public class MotionSynthesisComponent : MonoBehaviour, ISkeletonProvider
{
    [NonSerialized] public PoseBuffer CurrentPose;

    /// <summary>Layout of <see cref="CurrentPose"/> and the pose passed to every stage's Apply.</summary>
    public PoseLayout PoseLayout { get; private set; }

    /// <summary>Foot-contact Bool channels of the pipeline pose.</summary>
    public BoolHandle LeftFootContactHandle { get; private set; }
    public BoolHandle RightFootContactHandle { get; private set; }

    // Reused every LateUpdate as the mutable pose the stage chain runs on.
    private PoseBuffer _scratchPose;

    private SkeletonAsset _skeleton;
    public SkeletonAsset Skeleton => _skeleton;

    [SerializeField] [Tooltip("Optional skeleton asset binding for AnimationTools-based stages.")]
    private SkeletonAsset poseSkeleton;

    public SkeletonAsset PoseSkeleton => poseSkeleton;
    public SkeletonMap PoseSkeletonMap { get; private set; }

    /// <summary>
    /// The transforms of the character controlled by this <see cref="MotionSynthesisComponent"/>.
    /// These are the transforms that will be used to render the character.
    /// </summary>
    [NonSerialized] public Transform[] SkeletonTransforms;

    // TODO: remove?
    private Animator _animator;

    [Tooltip(
        "The frame rate of the animation synthesis. This is used to calculate the time step of the motion synthesis." +
        "Set to 0 for uncapped.")]
    public float synthesisFrameRate = 30f;

    [SerializeField] [Tooltip("The avatar that maps from mecanim template to character transforms.")]
    private Avatar avatar;

    [SerializeReference] [SubclassSelector]
    public List<MoSynthStage> stages = new();

    [Tooltip("Whether to animate the root position by Motion Matching or not.")]
    // maybe change this to 'root motion'?
    public bool rootPositionsMask = true;

    public float3 RootVelocity { get; protected set; }
    public float3 RootAngularVelocity { get; protected set; }
    public float3 RootPosition { get; protected set; }
    public quaternion RootRotation { get; protected set; }

    /// <summary>
    /// Time.DeltaTime if frame rate is not restricted. 1/animationFrameRate if restricted.
    /// </summary>
    private float _animationDeltaTime;

    bool IsFrameRateRestricted => synthesisFrameRate > 1e-5;

    private void Awake()
    {
        _animator = GetComponent<Animator>();

        _skeleton = null;

        if (IsFrameRateRestricted)
        {
            _animationDeltaTime = 1.0f / synthesisFrameRate;
        }

        stages.RemoveAll(stage => stage == null);
        foreach (var stage in stages)
        {
            _skeleton = stage.GetSkeleton(_skeleton);
        }

        if (_skeleton == null)
        {
            Debug.LogError($"MotionSynthesisComponent \"{name}\": no stage provided a skeleton " +
                           "(e.g. a MotionMatchingStage). Disabling the component.");
            enabled = false;
            return;
        }

        InitSkeletonTransformsArray();
        InitCurrentPose();
        ApplyTransformOffsetsFromSkeleton();

        if (poseSkeleton != null && _skeleton != null)
        {
            PoseSkeletonMap = SkeletonMap.Build(_skeleton, poseSkeleton);
            if (PoseSkeletonMap == null)
            {
                Debug.LogError(
                    $"MotionSynthesisComponent \"{name}\": could not map the MotionMatching skeleton onto poseSkeleton \"{poseSkeleton.name}\" (some joint has no matching bone by name or HumanBodyBones type).");
            }
        }

        foreach (var stage in stages)
        {
            stage.Init(this);
        }
    }


    private void InitSkeletonTransformsArray()
    {
        SkeletonTransforms = new Transform[_skeleton.BoneCount];
        SkeletonTransforms[0] = transform; // SimulationBone
        for (var i = 1; i < _skeleton.BoneCount; i++)
        {
            SkeletonTransforms[i] = _animator.GetBoneTransform(_skeleton.GetBone(i).humanBone);
        }
    }

    private void ApplyTransformOffsetsFromSkeleton()
    {
        for (var i = 1; i < _skeleton.BoneCount; i++)
        {
            var humanBone = _skeleton.GetBone(i).humanBone;
            var skeletonTransform = _animator.GetBoneTransform(humanBone);
            var skeletonBone = GetSkeletonBone(_animator.avatar.humanDescription, humanBone);
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

    private float _timeTillNextAnimationUpdate = 0f;

    private void LateUpdate()
    {
        if (!IsFrameRateRestricted)
        {
            _animationDeltaTime = Time.deltaTime;
        }
        else
        {
            _timeTillNextAnimationUpdate -= Time.deltaTime;
            if (_timeTillNextAnimationUpdate <= 0f)
            {
                _animationDeltaTime = 1f / synthesisFrameRate;
                _timeTillNextAnimationUpdate += _animationDeltaTime;
            }
            else
            {
                return;
            }
        }

        ConstructCurrentPoseFromSkeletonTransforms();

        _scratchPose.CopyFrom(CurrentPose);
        var pose = _scratchPose;
        foreach (var stage in stages)
        {
            if (stage.isEnabled)
            {
                if (!stage.Apply(pose, _animationDeltaTime))
                {
                    break;
                }
            }
        }

        ApplyPoseToSkeletonTransforms(pose);
    }

    void InitCurrentPose()
    {
        PoseLayout = PoseLayoutBuilder.Build(_skeleton, out var contacts);
        LeftFootContactHandle = contacts.Left;
        RightFootContactHandle = contacts.Right;

        CurrentPose = PoseBuffer.Allocate(PoseLayout, Allocator.Persistent);
        _scratchPose = PoseBuffer.Allocate(PoseLayout, Allocator.Persistent);

        var positions = CurrentPose.Positions;
        var rotations = CurrentPose.Rotations;
        for (var i = 0; i < SkeletonTransforms.Length; i++)
        {
            rotations[i] = SkeletonTransforms[i].localRotation;
            positions[i] = SkeletonTransforms[i].localPosition;
        }
        // Velocities, angular velocities and contacts are already zeroed by Allocate.
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

        var positions = CurrentPose.Positions;
        var rotations = CurrentPose.Rotations;
        var velocities = CurrentPose.Velocities;
        var angularVelocities = CurrentPose.AngularVelocities;

        for (var i = 0; i < angularVelocities.Length; i++)
        {
            var inverseLocalRotation = Quaternion.Inverse(rotations[i]);
            angularVelocities[i] =
                (SkeletonTransforms[i].localRotation * inverseLocalRotation).eulerAngles;
            velocities[i] =
                (float3)SkeletonTransforms[i].localPosition - positions[i];
        }

        positions[0] = SkeletonTransforms[0].localPosition;
        rotations[0] = SkeletonTransforms[0].localRotation;

        for (var i = 1; i < SkeletonTransforms.Length; i++)
        {
            rotations[i] = SkeletonTransforms[i].localRotation;
        }

        // hip
        positions[1] = SkeletonTransforms[1].localPosition;


        // CurrentPose.LeftFootContact = ?;
        // CurrentPose.RightFootContact = ?;
    }

    private void ApplyPoseToSkeletonTransforms(PoseBuffer pose)
    {
        var positions = pose.Positions;
        var rotations = pose.Rotations;

        // Motion
        // if (rootPositionsMask)
        // {
        //     // Motion Matching Root Motion + Floor Height
        //     Vector3 simulationBone = pose.JointLocalPositions[0];
        //     // simulationBone.y = _floorHeight;
        //     transform.position = simulationBone;
        // }

        for (var i = 1; i < _skeleton.BoneCount; i++)
        {
            SkeletonTransforms[i].localRotation = rotations[i];
        }

        // hips
        SkeletonTransforms[1].localPosition = positions[1];

        // root
        if (rootPositionsMask)
        {
            var rootTransform = SkeletonTransforms[0];
            rootTransform.localPosition +=
                rootTransform.localRotation * pose.Velocities[0] * _animationDeltaTime;
            var angVel = pose.AngularVelocities[0];
            var rootRotation = MathExtensions.QuaternionFromScaledAngleAxis(angVel * _animationDeltaTime);
            rootTransform.localRotation = rootRotation * rootTransform.localRotation;
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

        if (CurrentPose.IsCreated) CurrentPose.Dispose();
        if (_scratchPose.IsCreated) _scratchPose.Dispose();
    }


    public void SetRotAdjustment(quaternion adjustmentRotation)
    {
        throw new NotImplementedException();
    }

    public void SetPosAdjustment(float3 adjustmentPosition)
    {
        throw new NotImplementedException();
    }

    public float3 GetMainPositionFeature(int trajectoryIndex)
    {
        throw new NotImplementedException();
    }

    public float4 GetEnvironmentFeature(string featureName, int trajectoryIndex)
    {
        throw new NotImplementedException();
    }
}
}