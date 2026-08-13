using System;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
[Serializable]
public class RetargetingStage : MoSynthStage
{
    private MotionSynthesisComponent _owner;

    /// <summary>
    /// The T-pose obtained from one of the animation assets.
    /// </summary>
    private quaternion[] _animationTPose;

    /// <summary>
    /// The T-pose of the character skeleton set up during the Mecanim mapping.   
    /// </summary>
    private quaternion[] _characterTPose;

    // TODO: this requires that an animator component be set up
    // find another way to retarget
    private Animator _animator;

    // TODO: Retargeting should be independent of Motion Matching
    public MotionMatchingData mmData;

    private quaternion _hipsCorrection;

    public override void Init(MotionSynthesisComponent motionSynthesisComponent)
    {
        _owner = motionSynthesisComponent;
        _animator = motionSynthesisComponent.GetComponent<Animator>();

        // TODO: use only one list of joints
        // currently animJoints dont have Joint.type set correctly
        // and poseJoints have extra simBone entry
        var poseSet = mmData.GetOrImportPoseSet();
        var poseJoints = poseSet.Skeleton.Joints;

        var tPoseAnimation = mmData.tPoseAnimationClip;
        var boneCount = tPoseAnimation.Skeleton.BoneCount;

        _animationTPose = new quaternion[boneCount];
        _characterTPose = new quaternion[boneCount];

        for (var i = 0; i < boneCount; i++)
        {
            _animationTPose[i] = tPoseAnimation.GetWorldRotation(i, 0);
        }

        var tPoseSkeleton = _animator.avatar.humanDescription.skeleton;
        for (var i = 0; i < boneCount; i++)
        {
            var jointTransform = _animator.GetBoneTransform(poseJoints[i + 1].type);
            var targetJointIndex = Array.FindIndex(tPoseSkeleton, bone => bone.name == jointTransform.name);
            Debug.Assert(targetJointIndex != -1, "Target joint not found: " + jointTransform.name);

            // Initialize the rotation as the local rotation of the joint
            var cumulativeRotation = tPoseSkeleton[targetJointIndex].rotation;

            // Traverse up the hierarchy until reaching the Animator's transform
            var currentTransform = jointTransform.parent;
            while (currentTransform != null && currentTransform != _animator.transform)
            {
                var parentIndex = Array.FindIndex(tPoseSkeleton, bone => bone.name == currentTransform.name);
                if (parentIndex != -1)
                {
                    // Multiply with the parent's local rotation
                    cumulativeRotation = tPoseSkeleton[parentIndex].rotation * cumulativeRotation;
                }

                Debug.Assert(parentIndex != -1, "Parent joint not found: " + currentTransform.name);

                // Move to the next parent in the hierarchy
                currentTransform = currentTransform.parent;
            }

            // Store the world rotation
            _characterTPose[i] = cumulativeRotation;
        }

        var sourceWorldForward = math.mul(_animationTPose[0], mmData.hipsForwardLocalVector);
        var sourceWorldUp = math.mul(_animationTPose[0], mmData.hipsUpLocalVector);
        var sourceLookAt = quaternion.LookRotation(sourceWorldForward, sourceWorldUp);
        _hipsCorrection = sourceLookAt;
    }


    public override bool Apply(PoseVector pose, float deltaTime)
    {
        var animationSkeleton = _owner.Skeleton;

        var sourcePose = new PoseVector(pose);

        // Retargeting
        for (var i = 0; i < animationSkeleton.Joints.Count - 1; i++)
        {
            // Motion Matching Target Rotation
            var sourceTPoseRotation = _animationTPose[i];
            var targetTPoseRotation = _characterTPose[i];

            // TODO: precalculate for entire skeleton

            var newSourceRot =
                animationSkeleton.GetRootSpaceRotation(animationSkeleton.Joints[i + 1], sourcePose);

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

            var newTargetRot =
                math.mul(
                    math.mul(newSourceRot, math.inverse(sourceTPoseRotation)),
                    math.mul(_hipsCorrection, targetTPoseRotation)
                );


            var parentJoint = animationSkeleton.GetParent(animationSkeleton.Joints[i + 1]);
            var parentRot = animationSkeleton.GetRootSpaceRotation(parentJoint, pose);

            var newTargetLocalRot = Quaternion.Inverse(parentRot) * newTargetRot;

            pose.jointLocalRotations[i + 1] = newTargetLocalRot;
        }

        return true;

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
    }
}
}