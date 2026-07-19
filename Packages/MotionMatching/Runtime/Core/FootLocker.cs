// using Unity.Mathematics;
// using UnityEngine;
// using System;
//
// namespace MotionMatching
// {
// [Serializable]
// public class FootLocker
// {
//     public bool isEnabled;
//     
//     // Foot Lock
//     public bool IsLeftFootContact { get; private set; }
//
//     public bool IsRightFootContact { get; private set; }
//
//     // Target position of the toes
//     public float3 LeftToesContactTarget { get; private set; }
//     public float3 RightToesContactTarget { get; private set; }
//     private float3 _leftFootContact, _rightFootContact; // Position of the foot
//     private float3 _leftFootPoleContact, _rightFootPoleContact; // Forward vector of the knee
//     private float3 _leftLowerLegLocalForward, _rightLowerLegLocalForward;
//     public int LeftToesIndex { get; private set; }
//     private int _leftFootIndex;
//     private int _leftLowerLegIndex;
//     private int _leftUpperLegIndex;
//
//     public int RightToesIndex { get; private set; }
//     private int _rightFootIndex;
//     private int _rightLowerLegIndex;
//     private int _rightUpperLegIndex;
//
//     public float footUnlockDistance = 0.2f; // Distance from actual pose to IK target to unlock the feet
//     private float _contactVelocityThreshold;
//
//
//     public void Initialize(MotionMatchingData mmData)
//     {
//         var skeleton = mmData.GetOrImportPoseSet().Skeleton;
//         LeftToesIndex = skeleton.GetJointIndex(HumanBodyBones.LeftToes);
//         _leftFootIndex = skeleton.GetJointIndex(HumanBodyBones.LeftFoot);
//         _leftLowerLegIndex = skeleton.GetJointIndex(HumanBodyBones.LeftLowerLeg);
//         _leftUpperLegIndex = skeleton.GetJointIndex(HumanBodyBones.LeftUpperLeg);
//         RightToesIndex = skeleton.GetJointIndex(HumanBodyBones.RightToes);
//         _rightFootIndex = skeleton.GetJointIndex(HumanBodyBones.RightFoot);
//         _rightLowerLegIndex = skeleton.GetJointIndex(HumanBodyBones.RightLowerLeg);
//         _rightUpperLegIndex = skeleton.GetJointIndex(HumanBodyBones.RightUpperLeg);
//         _leftLowerLegLocalForward = mmData.GetLocalForward(_leftLowerLegIndex);
//         _rightLowerLegLocalForward = mmData.GetLocalForward(_rightLowerLegIndex);
//
//         _contactVelocityThreshold = mmData.ContactVelocityThreshold;
//     }
//     
//     public void Apply(Transform[] skeletonTransforms, PoseVector pose)
//     {
//         float3 currentLeftToesPosition = skeletonTransforms[LeftToesIndex].position;
//         float3 currentRightToesPosition = skeletonTransforms[RightToesIndex].position;
//         // Compute input contact position velocity
//         var currentLeftToesVelocity = (currentLeftToesPosition - LeftToesContactTarget) / Time.deltaTime;
//         var currentRightToesVelocity = (currentRightToesPosition - RightToesContactTarget) / Time.deltaTime;
//         LeftToesContactTarget = currentLeftToesPosition;
//         RightToesContactTarget = currentRightToesPosition;
//
//         // Update Inertializer
//         _inertialization.UpdateContact(IsLeftFootContact ? _leftFootContact : currentLeftToesPosition,
//             IsLeftFootContact ? float3.zero : currentLeftToesVelocity,
//             IsRightFootContact ? _rightFootContact : currentRightToesPosition,
//             IsRightFootContact ? float3.zero : currentRightToesVelocity,
//             inertializeHalfLife, Time.deltaTime);
//         float3 leftContactPosition = _inertialization.InertializedLeftContact;
//         float3 leftContactVelocity = _inertialization.InertializedLeftContactVelocity;
//         float3 rightContactPosition = _inertialization.InertializedRightContact;
//         float3 rightContactVelocity = _inertialization.InertializedRightContactVelocity;
//
//         // If the contact point is too far from the current input position
//         // unlock the contact
//         var unlockLeftContact = IsLeftFootContact &&
//                                 (math.length(_leftFootContact - currentLeftToesPosition) > footUnlockDistance);
//         var unlockRightContact = IsRightFootContact &&
//                                  (math.length(_rightFootContact - currentRightToesPosition) > footUnlockDistance);
//
//         // If the contact was previously inactive and now it is active,
//         // transition to the locked contact state
//         // Also, make sure the inertialization returns an almost 0 velocity before locking
//         if (!IsLeftFootContact && pose.LeftFootContact &&
//             math.length(leftContactVelocity) < _contactVelocityThreshold)
//         {
//             // Contact point is the current position of the foot
//             // projected onto the ground + foot height
//             IsLeftFootContact = true;
//             _leftFootContact = leftContactPosition;
//             // LeftFootContact.y =  // TODO: Add foot height
//             var leftLowerLeg = skeletonTransforms[_leftLowerLegIndex];
//             _leftFootPoleContact = math.mul(leftLowerLeg.rotation, _leftLowerLegLocalForward);
//
//             if (inertialize)
//             {
//                 _inertialization.LeftContactTransition(currentLeftToesPosition, currentLeftToesVelocity,
//                     _leftFootContact, float3.zero);
//             }
//             else
//             {
//                 _inertialization.LeftContactTransition(currentLeftToesPosition, currentLeftToesVelocity,
//                     currentLeftToesPosition, currentLeftToesVelocity);
//             }
//         }
//         // If we need to unlock or previously in contact but now not
//         // we transition to the input position
//         else if (unlockLeftContact || (IsLeftFootContact && !pose.LeftFootContact))
//         {
//             IsLeftFootContact = false;
//
//             if (inertialize)
//             {
//                 _inertialization.LeftContactTransition(_leftFootContact, float3.zero, currentLeftToesPosition,
//                     currentLeftToesVelocity);
//             }
//             else
//             {
//                 _inertialization.LeftContactTransition(currentLeftToesPosition, currentLeftToesVelocity,
//                     currentLeftToesPosition, currentLeftToesVelocity);
//             }
//         }
//
//         // Same for Right Foot
//         if (!IsRightFootContact && pose.RightFootContact &&
//             math.length(rightContactVelocity) < _contactVelocityThreshold)
//         {
//             IsRightFootContact = true;
//             _rightFootContact = rightContactPosition;
//             // RightFootContact.y = 0.0f;
//             var rightLowerLeg = skeletonTransforms[_rightLowerLegIndex];
//             _rightFootPoleContact = math.mul(rightLowerLeg.rotation, _rightLowerLegLocalForward);
//
//             if (inertialize)
//             {
//                 _inertialization.RightContactTransition(currentRightToesPosition, currentRightToesVelocity,
//                     _rightFootContact, float3.zero);
//             }
//             else
//             {
//                 _inertialization.RightContactTransition(currentRightToesPosition, currentRightToesVelocity,
//                     currentRightToesPosition, currentRightToesVelocity);
//             }
//         }
//         else if (unlockRightContact || (IsRightFootContact && !pose.RightFootContact))
//         {
//             IsRightFootContact = false;
//
//             if (inertialize)
//             {
//                 _inertialization.RightContactTransition(_rightFootContact, float3.zero, currentRightToesPosition,
//                     currentRightToesVelocity);
//             }
//             else
//             {
//                 _inertialization.RightContactTransition(currentRightToesPosition, currentRightToesVelocity,
//                     currentRightToesPosition, currentRightToesVelocity);
//             }
//         }
//
//         // IK to place the foot
//         if (isEnabled)
//         {
//             // Left Foot IK
//             TwoJointIK.Solve(
//                 (Vector3)leftContactPosition + (skeletonTransforms[_leftFootIndex].position -
//                                                 skeletonTransforms[LeftToesIndex].position),
//                 skeletonTransforms[_leftUpperLegIndex],
//                 skeletonTransforms[_leftLowerLegIndex],
//                 skeletonTransforms[_leftFootIndex],
//                 _leftFootPoleContact);
//             // Right Foot IK
//             TwoJointIK.Solve(
//                 (Vector3)rightContactPosition + (skeletonTransforms[_rightFootIndex].position -
//                                                  skeletonTransforms[RightToesIndex].position),
//                 skeletonTransforms[_rightUpperLegIndex],
//                 skeletonTransforms[_rightLowerLegIndex],
//                 skeletonTransforms[_rightFootIndex],
//                 _rightFootPoleContact);
//         }
//     }
// }
// }