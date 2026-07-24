using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System;
using System.Linq;

namespace MotionMatching
{
[RequireComponent(typeof(Animator))]
public class MotionMatchingSkinnedMeshRenderer : MonoBehaviour
{
    [Header("General")] 
    public MotionMatchingController motionMatching;

    [Header("Animator Integration")]
    
    [Tooltip("Joints animated by Motion Matching. If none, all Joints are animated.")]
    public AvatarMaskData avatarMask;

    [Tooltip("Whether to animate the root position by Motion Matching or not.")]
    public bool rootPositionsMask = true;

    [Tooltip("Whether to animate the root rotations by Motion Matching or not.")]
    public bool rootRotationsMask = true;

    [Tooltip("Whether poses should be blended when a joint changes its animation source (Motion Matching or Unity's Animator)")]
    public bool blendPoses = true;

    [Tooltip("Decrease this value to accelerate blending. Time needed to move half of the distance between the source to the target pose.")]
    [Range(0.0f, 1.0f)]
    public float blendHalfLife = 0.05f;

    [Header("Toes Floor Penetration")]
    [Tooltip(
        "Enable to avoid the toes joint (+ ToesSoleOffset) to penetrate the floor (assuming floor at y=0). The root joint will be adjusted to compensate the height difference.")]
    public bool avoidToesFloorPenetration;

    [Tooltip("Offset added to the toes joint to determine the sole position to avoid toes-floor penetration.")]
    public Vector3 toesSoleOffset;

    // References
    private Animator _animator;

    // Retargeting
    // Initial orientations of the bones. The code assumes the initial orientations are in T-Pose
    private Quaternion[] _animationTPose;

    private Quaternion[] _characterTPose;

    // Mapping from BodyJoints to the actual transforms
    private Transform[] _sourceBones;

    private Transform[] _targetBones;

    // Mapping Hips Orientation
    private Quaternion _hipsCorrection;

    // Toes-Floor Penetration
    private float _toesPenetrationMovingCorrection;

    // Height
    private float _floorHeight;

    // Inertialization
    private bool[] _previousJointMask;
    private bool _previousHipsPositionMask;
    private quaternion[] _previousJointRotations;
    private quaternion[] _offsetJointRotations;
    private float3 _previousHipsPosition;
    private float3 _offsetHipsPosition;

    private void Awake()
    {
        _animator = GetComponent<Animator>();
        _previousJointMask = new bool[BodyJoints.Length];
        _previousJointRotations = new quaternion[BodyJoints.Length];
        _offsetJointRotations = new quaternion[BodyJoints.Length];
    }

    private void OnEnable()
    {
        motionMatching.OnSkeletonTransformUpdated += OnSkeletonTransformUpdated;

        UpdatePreviousInertialization();
    }

    private void OnDisable()
    {
        motionMatching.OnSkeletonTransformUpdated -= OnSkeletonTransformUpdated;
    }

    private void Start()
    {
        InitRetargeting();
    }

    public void SetFloorHeight(float floorY)
    {
        _floorHeight = floorY;
    }

    private void InitRetargeting()
    {
        var mmData = motionMatching.mmData;
        _animationTPose = new Quaternion[BodyJoints.Length];
        _characterTPose = new Quaternion[BodyJoints.Length];
        _sourceBones = new Transform[BodyJoints.Length];
        _targetBones = new Transform[BodyJoints.Length];
        // Animation containing in the first frame a T-Pose
        var tPoseAnimation = mmData.AnimationDataTPose.GetAnimation();
        // Store Rotations
        // Source
        var skeleton = tPoseAnimation.Skeleton;
        for (var i = 0; i < BodyJoints.Length; i++)
        {
            if (mmData.TryGetJointName(BodyJoints[i], out var jointName) &&
                skeleton.TryFind(jointName, out var joint))
            {
                // Get the rotation for the first frame of the animation
                _animationTPose[i] = tPoseAnimation.GetWorldRotation(joint, 0);
            }
        }

        // Target
        var rot = _animator.transform.rotation;
        _animator.transform.rotation = Quaternion.identity;
        var targetSkeletonBones = _animator.avatar.humanDescription.skeleton;
        var targetHipsRot = Quaternion.identity;
        for (var i = 0; i < BodyJoints.Length; i++)
        {
            var targetJoint = _animator.GetBoneTransform(BodyJoints[i]);

            // Use Array.FindIndex to find the index of the joint in the targetSkeletonBones array
            var targetJointIndex = Array.FindIndex(targetSkeletonBones, bone => bone.name == targetJoint.name);
            Debug.Assert(targetJointIndex != -1, "Target joint not found: " + targetJoint.name);

            // Initialize the rotation as the local rotation of the joint
            var cumulativeRotation = targetSkeletonBones[targetJointIndex].rotation;

            // Traverse up the hierarchy until reaching the Animator's transform
            var currentTransform = targetJoint.parent;
            while (currentTransform != null && currentTransform != _animator.transform)
            {
                var parentIndex = Array.FindIndex(targetSkeletonBones, bone => bone.name == currentTransform.name);
                if (parentIndex != -1)
                {
                    // Multiply with the parent's local rotation
                    cumulativeRotation = targetSkeletonBones[parentIndex].rotation * cumulativeRotation;
                }

                Debug.Assert(parentIndex != -1, "Parent joint not found: " + currentTransform.name);

                // Move to the next parent in the hierarchy
                currentTransform = currentTransform.parent;
            }

            // Store the world rotation
            _characterTPose[i] = cumulativeRotation;
            if (BodyJoints[i] == HumanBodyBones.Hips)
            {
                targetHipsRot = cumulativeRotation;
            }
        }

        _animator.transform.rotation = rot;
        // Find ForwardLocalVector and UpLocalVector
        var forwardLocalVector = math.mul(math.inverse(targetHipsRot), math.forward());
        var upLocalVector = math.mul(math.inverse(targetHipsRot), math.up());
        // Correct body orientation so they are both facing the same direction
        var targetWorldForward = math.mul(_characterTPose[0], forwardLocalVector);
        var targetWorldUp = math.mul(_characterTPose[0], upLocalVector);
        var sourceWorldForward = math.mul(_animationTPose[0], mmData.HipsForwardLocalVector);
        var sourceWorldUp = math.mul(_animationTPose[0], mmData.HipsUpLocalVector);
        var targetLookAt = quaternion.LookRotation(targetWorldForward, targetWorldUp);
        var sourceLookAt = quaternion.LookRotation(sourceWorldForward, sourceWorldUp);
        _hipsCorrection = math.mul(sourceLookAt, math.inverse(targetLookAt));
        // Store Transforms
        var mmBones = motionMatching.SkeletonTransforms;

        // Source
        for (var i = 0; i < BodyJoints.Length; i++)
        {
            if (mmData.TryGetJointName(BodyJoints[i], out var jointName))
            {
                _sourceBones[i] = mmBones.FirstOrDefault(b => b.name == jointName);
            }
        }

        // Target
        for (var i = 0; i < BodyJoints.Length; i++)
        {
            _targetBones[i] = _animator.GetBoneTransform(BodyJoints[i]);
        }
    }

    private void OnSkeletonTransformUpdated()
    {
        // Motion
        if (rootPositionsMask)
        {
            // Motion Matching Root Motion + Floor Height
            var simulationBone = motionMatching.transform.position;
            simulationBone.y = _floorHeight;
            transform.position = simulationBone;
        }

        motionMatching.SetPosAdjustment(transform.position - motionMatching.transform.position);

        // Retargeting
        for (var i = 0; i < BodyJoints.Length; i++)
        {
            var currentJointMask = false;
            // Unity's Animator Target Rotation
            var targetRotation = _targetBones[i].rotation;
            // Check Avatar Mask
            if (avatarMask == null ||
                (BodyJoints[i] == HumanBodyBones.Hips && rootRotationsMask) ||
                (BodyJoints[i] != HumanBodyBones.Hips && avatarMask != null && avatarMask.IsEnabled(BodyJoints[i])))
            {
                currentJointMask = true;
                // Motion Matching Target Rotation
                var sourceTPoseRotation = _animationTPose[i];
                var targetTPoseRotation = _characterTPose[i];
                var sourceRotation = _sourceBones[i].rotation;
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
                targetRotation = sourceRotation * Quaternion.Inverse(sourceTPoseRotation) * _hipsCorrection *
                                 targetTPoseRotation;
                // targetRotation = sourceRotation;
            }

            if (blendPoses && _previousJointMask[i] != currentJointMask)
            {
                // Pose Transition
                var offsetAngVel = float3.zero;
                Inertialization.InertializeJointTransition(_previousJointRotations[i], float3.zero,
                    targetRotation, float3.zero,
                    ref _offsetJointRotations[i], ref offsetAngVel);
            }

            if (blendPoses)
            {
                var offsetAngVel = float3.zero;
                Inertialization.InertializeJointUpdate(targetRotation, float3.zero,
                    blendHalfLife, Time.deltaTime,
                    ref _offsetJointRotations[i], ref offsetAngVel,
                    out var inertializedRotation, out _);
                _targetBones[i].rotation = inertializedRotation;
            }
            else
            {
                _targetBones[i].rotation = targetRotation;
            }
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
            var offsetAngVel = float3.zero;
            Inertialization.InertializeJointTransition(_previousHipsPosition, float3.zero,
                targetHipsPosition, float3.zero,
                ref _offsetHipsPosition, ref offsetAngVel);
        }

        if (blendPoses)
        {
            var offsetAngVel = float3.zero;
            Inertialization.InertializeJointUpdate(targetHipsPosition, float3.zero,
                blendHalfLife, Time.deltaTime,
                ref _offsetHipsPosition, ref offsetAngVel,
                out var inertializedHipsPosition, out _);
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
            var soleHeightOffset = Mathf.Min(_targetBones[leftToesIndex].TransformPoint(toesSoleOffset).y,
                _targetBones[rightToesIndex].TransformPoint(toesSoleOffset).y);
            soleHeightOffset = soleHeightOffset < _floorHeight ? -soleHeightOffset : 0.0f;

            const float movingAverageFactor = 0.99f;
            _toesPenetrationMovingCorrection = _toesPenetrationMovingCorrection * movingAverageFactor +
                                               (soleHeightOffset + _floorHeight) * (1.0f - movingAverageFactor);

            var hipsPos = _targetBones[0].position;
            hipsPos.y += _toesPenetrationMovingCorrection;
            _targetBones[0].position = hipsPos;
        }

        // Update State
        UpdatePreviousInertialization();
    }

    private void UpdatePreviousInertialization()
    {
        // Previous Joint Mask
        _previousJointMask[0] = rootRotationsMask;
        for (var i = 1; i < BodyJoints.Length; i++)
        {
            _previousJointMask[i] = !avatarMask || avatarMask.IsEnabled(BodyJoints[i]);
        }

        _previousHipsPositionMask = rootPositionsMask;
        // Previous Joint Rotations
        for (var i = 0; i < _previousJointRotations.Length; ++i)
            _previousJointRotations[i] = _targetBones != null ? _targetBones[i].rotation : quaternion.identity;
        _previousHipsPosition = _targetBones != null ? _targetBones[0].position : float3.zero;
    }

    // Used for retargeting. First parent, then children
    public static readonly HumanBodyBones[] BodyJoints =
    {
        HumanBodyBones.Hips, // 0

        HumanBodyBones.Spine, // 1
        HumanBodyBones.Chest, // 2
        HumanBodyBones.UpperChest, // 3

        HumanBodyBones.Neck, // 4
        HumanBodyBones.Head, // 5

        HumanBodyBones.LeftShoulder, // 6
        HumanBodyBones.LeftUpperArm, // 7
        HumanBodyBones.LeftLowerArm, // 8
        HumanBodyBones.LeftHand, // 9

        HumanBodyBones.RightShoulder, // 10
        HumanBodyBones.RightUpperArm, // 11
        HumanBodyBones.RightLowerArm, // 12
        HumanBodyBones.RightHand, // 13

        HumanBodyBones.LeftUpperLeg, // 14
        HumanBodyBones.LeftLowerLeg, // 15
        HumanBodyBones.LeftFoot, // 16
        HumanBodyBones.LeftToes, // 17

        HumanBodyBones.RightUpperLeg, // 18
        HumanBodyBones.RightLowerLeg, // 19
        HumanBodyBones.RightFoot, // 20
        HumanBodyBones.RightToes // 21
    };

#if UNITY_EDITOR
    private void OnDrawGizmosSelected()
    {
        if (!enabled) return;
        var animator = GetComponent<Animator>();

        if (animator == null) return;

        var leftSole = animator.GetBoneTransform(HumanBodyBones.LeftToes).TransformPoint(toesSoleOffset);
        var rightSole = animator.GetBoneTransform(HumanBodyBones.RightToes).TransformPoint(toesSoleOffset);
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(leftSole, 0.005f);
        Gizmos.DrawSphere(rightSole, 0.005f);
    }
#endif
}
}