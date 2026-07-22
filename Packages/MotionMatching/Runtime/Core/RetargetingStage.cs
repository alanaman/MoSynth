
using System;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
[Serializable]
public class RetargetingStage : MoSynthStage
{
    private MotionSynthesisComponent _owner;
    
    Quaternion[] _characterTPose;
    Quaternion[] _templateTPose;
    
    // TODO: this requires that an animator component be set up
    // find another way to retarget
    private Animator _animator;
    
    // TODO: Retargeting should be independent of Motion Matching
    public MotionMatchingData mmData;
    
    private Quaternion _hipsCorrection;
    
    public override void Init(MotionSynthesisComponent motionSynthesisComponent)
    {
        var bodyJoints = MotionMatchingSkinnedMeshRenderer.BodyJoints;
        _owner = motionSynthesisComponent;
        _animator = motionSynthesisComponent.GetComponent<Animator>();
        
        _characterTPose = new Quaternion[bodyJoints.Length];
        _templateTPose = new Quaternion[bodyJoints.Length];
        
        var tPoseAnimation = mmData.AnimationDataTPose.GetAnimation();
        var skeleton = tPoseAnimation.Skeleton;
        for (var i = 0; i < bodyJoints.Length; i++)
        {
            if (mmData.TryGetJointName(bodyJoints[i], out var jointName) &&
                skeleton.TryFind(jointName, out var joint))
            {
                // Get the rotation for the first frame of the animation
                _characterTPose[i] = tPoseAnimation.GetWorldRotation(joint, 0);
            }
        }
        
        var templateSkeleton = _animator.avatar.humanDescription.skeleton;
        var targetHipsRot = Quaternion.identity;
        for (var i = 0; i < bodyJoints.Length; i++)
        {
            var jointTransform = _animator.GetBoneTransform(bodyJoints[i]);
            var targetJointIndex = Array.FindIndex(templateSkeleton, bone => bone.name == jointTransform.name);
            Debug.Assert(targetJointIndex != -1, "Target joint not found: " + jointTransform.name);
            
            // Initialize the rotation as the local rotation of the joint
            var cumulativeRotation = templateSkeleton[targetJointIndex].rotation;
            
            // Traverse up the hierarchy until reaching the Animator's transform
            var currentTransform = jointTransform.parent;
            while (currentTransform != null && currentTransform != _animator.transform)
            {
                var parentIndex = Array.FindIndex(templateSkeleton, bone => bone.name == currentTransform.name);
                if (parentIndex != -1)
                {
                    // Multiply with the parent's local rotation
                    cumulativeRotation = templateSkeleton[parentIndex].rotation * cumulativeRotation;
                }

                Debug.Assert(parentIndex != -1, "Parent joint not found: " + currentTransform.name);

                // Move to the next parent in the hierarchy
                currentTransform = currentTransform.parent;
            }
            
            // Store the world rotation
            _templateTPose[i] = cumulativeRotation;
            if (bodyJoints[i] == HumanBodyBones.Hips)
            {
                targetHipsRot = cumulativeRotation;
            }
        }
        
        // Find ForwardLocalVector and UpLocalVector
        var forwardLocalVector = math.mul(math.inverse(targetHipsRot), math.forward());
        var upLocalVector = math.mul(math.inverse(targetHipsRot), math.up());
        // Correct body orientation so they are both facing the same direction
        var targetWorldForward = math.mul(_templateTPose[0], forwardLocalVector);
        var targetWorldUp = math.mul(_templateTPose[0], upLocalVector);
        var sourceWorldForward = math.mul(_characterTPose[0], mmData.HipsForwardLocalVector);
        var sourceWorldUp = math.mul(_characterTPose[0], mmData.HipsUpLocalVector);
        var targetLookAt = quaternion.LookRotation(targetWorldForward, targetWorldUp);
        var sourceLookAt = quaternion.LookRotation(sourceWorldForward, sourceWorldUp);
        _hipsCorrection = math.mul(sourceLookAt, math.inverse(targetLookAt));
    }


    public override void Apply(PoseVector pose, float deltaTime)
    {
        // motionMatching.SetPosAdjustment(transform.position - motionMatching.transform.position);
        
        var skeleton = _owner.Skeleton;
        var bodyJoints = MotionMatchingSkinnedMeshRenderer.BodyJoints;
        
        // Retargeting
        for (var i = 0; i < skeleton.Joints.Count; i++)
        {
            // Motion Matching Target Rotation
            var sourceTPoseRotation = _characterTPose[i];
            var targetTPoseRotation = _templateTPose[i];
            // var newRot = pose.GetWorldSpaceRotation(skeleton, skeleton.Joints[i]);
            var newLocalRot = pose.JointLocalRotations[skeleton.Joints[i].index];
            
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

            pose.JointLocalRotations[skeleton.Joints[i].index] =
                newLocalRot * Quaternion.Inverse(sourceTPoseRotation) *
                targetTPoseRotation * _hipsCorrection;
        }

        // Hips
        // if (rootPositionsMask)
        // {
        //     _targetBones[0].position = (float3)owner.SkeletonTransforms[1].position;
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
    }
}
}