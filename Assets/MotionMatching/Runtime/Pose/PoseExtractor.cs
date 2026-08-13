using UnityEngine;
using Unity.Mathematics;
using System;
using AnimationTools;

namespace MotionMatching
{
/// <summary>
/// Extracts full pose for Motion Matching from BVHAnimation
/// </summary>
public static class PoseExtractor
{
    /// <summary>
    /// Extract the poses from bvhAnimation and store it in poseSet
    /// poseSet is not cleared, it will add bvhAnimation the the existing poses
    /// Returns true if the bvhAnimation was added to the poseSet, false otherwise
    /// </summary>
    public static bool Extract(AnnotatedAnimationClip animationClip, PoseSet poseSet, IPoseSetSource source)
    {
        // Set Poses
        var nFrames = animationClip.FrameCount;
        var poses = new PoseVector[nFrames - 1];
        var nBvhJoints = animationClip.Skeleton.BoneCount;
        var nPoseSetJoints = nBvhJoints + 1; // +1 for SimulationBone

        if (!animationClip.Skeleton.TryFindByHumanBone(HumanBodyBones.LeftToes, out var leftToesBoneIndex))
        {
            Debug.LogError("LeftToes not found in BVHAnimation");
            leftToesBoneIndex = 0; // legacy TryFind left a default joint (index 0) on failure
        }

        var leftToesIndex = leftToesBoneIndex + 1; // +1 for SimulationBone
        if (!animationClip.Skeleton.TryFindByHumanBone(HumanBodyBones.RightToes, out var rightToesBoneIndex))
        {
            Debug.LogError("RightToes not found in BVHAnimation");
            rightToesBoneIndex = 0; // legacy TryFind left a default joint (index 0) on failure
        }

        var rightToesIndex = rightToesBoneIndex + 1; // +1 for SimulationBone

        // Legacy contact-extraction path (GetWorldSpaceVelocity) walks a MotionMatching.Skeleton's
        // Joint parent chain, unshifted (no SimulationBone) — mirror the SkeletonAsset bone list
        // into that legacy shape rather than touching ExtractPoseContacts itself.
        var bvhSkeleton = new Skeleton();
        for (var i = 0; i < nBvhJoints; i++)
        {
            var bone = animationClip.Skeleton.GetBone(i);
            bvhSkeleton.AddJoint(new Skeleton.Joint(bone.name, i, bone.parentIndex, bone.restLocalPosition, bone.humanBone));
        }

        for (var i = 0; i < nFrames - 1; i++)
        {
            poses[i] = new PoseVector(nPoseSetJoints);
            ExtractPose(ref poses[i], animationClip, i, source);
        }

        for (var i = 0; i < nFrames - 2; i++)
        {
            ExtractPoseVelocities(ref poses[i], poses[i + 1], animationClip);
        }
        var lastPose = new PoseVector(nPoseSetJoints);
        ExtractPose(ref lastPose, animationClip, nFrames - 1, source);
        ExtractPoseVelocities(ref poses[^1], lastPose, animationClip);

        for (var i = 0; i < nFrames - 1; i++)
        {
            // Note: this requires velocities to be pre-calculated
            ExtractPoseContacts(ref poses[i],
                bvhSkeleton,
                leftToesIndex,
                rightToesIndex,
                source.ContactVelocityThreshold);
        }


        SmoothContacts(poses);

        if (poseSet.AddClip(poses, animationClip.FrameTime, animationClip.tags))
        {
            return true;
        }

        return false;
    }

    private static void SmoothContacts(PoseVector[] poses)
    {
        const int windowsRadius = 6;
        // Median filter to remove small regions where contact is either active or inactive
        var leftFootContact = new bool[poses.Length];
        var rightFootContact = new bool[poses.Length];
        for (var i = 0; i < poses.Length; i++)
        {
            leftFootContact[i] = poses[i].leftFootContact;
            rightFootContact[i] = poses[i].rightFootContact;
        }

        // Median Filter
        Span<bool> leftFootContactWindow = stackalloc bool[windowsRadius * 2 + 1];
        Span<bool> rightFootContactWindow = stackalloc bool[windowsRadius * 2 + 1];
        for (var i = 0; i < poses.Length; i++)
        {
            var pose = poses[i];
            var windowIndex = 0;
            for (var j = -windowsRadius; j <= windowsRadius; j++)
            {
                var index = i + j;
                if (index < 0)
                {
                    leftFootContactWindow[windowIndex] = leftFootContact[0];
                    rightFootContactWindow[windowIndex] = rightFootContact[0];
                }
                else if (index >= poses.Length)
                {
                    leftFootContactWindow[windowIndex] = leftFootContact[poses.Length - 1];
                    rightFootContactWindow[windowIndex] = rightFootContact[poses.Length - 1];
                }
                else
                {
                    leftFootContactWindow[windowIndex] = leftFootContact[index];
                    rightFootContactWindow[windowIndex] = rightFootContact[index];
                }

                windowIndex += 1;
            }

            // Sort
            var lastFalseIndex = 0;
            for (var j = 0; j < windowsRadius * 2 + 1; j++)
            {
                if (!leftFootContactWindow[j])
                {
                    var aux = leftFootContactWindow[lastFalseIndex];
                    leftFootContactWindow[lastFalseIndex] = false;
                    leftFootContactWindow[j] = aux;
                    lastFalseIndex += 1;
                }
            }

            lastFalseIndex = 0;
            for (var j = 0; j < windowsRadius * 2 + 1; j++)
            {
                if (!rightFootContactWindow[j])
                {
                    var aux = rightFootContactWindow[lastFalseIndex];
                    rightFootContactWindow[lastFalseIndex] = false;
                    rightFootContactWindow[j] = aux;
                    lastFalseIndex += 1;
                }
            }

            // Find median
            var medianIndex = windowsRadius;
            pose.leftFootContact = leftFootContactWindow[medianIndex];
            pose.rightFootContact = rightFootContactWindow[medianIndex];
            poses[i] = pose;
        }
    }

    private static void ExtractPose(ref PoseVector pose, AnnotatedAnimationClip animationClip, int frameIndex,
        IPoseSetSource source)
    {
        var frame = animationClip.GetFrame(frameIndex);
        var rotations = frame.Rotations;
        var skeleton = animationClip.Skeleton;

        // Joints
        for (var i = 1; i < pose.jointLocalPositions.Length; i++)
        {
            // rest offsets come from the skeleton, not the frame's positions section (slot 0 there is root motion)
            pose.jointLocalPositions[i] = skeleton.GetBone(i - 1).restLocalPosition;
            pose.jointLocalRotations[i] = rotations[i - 1];
        }

        // SimulationBone
        // position and direction are hips projected on the ground
        Vector3 frameRootMotion = (float3)frame.Positions[0];
        var sbPos = new Vector3(frameRootMotion.x, 0.0f, frameRootMotion.z);
        Vector3 hipsForwardDir = (Quaternion)rotations[0] * (Vector3)(float3)source.HipsForwardLocalVector;
        hipsForwardDir.y = 0;
        hipsForwardDir = hipsForwardDir.normalized;
        var sbRot = Quaternion.LookRotation(hipsForwardDir, Vector3.up);
        pose.jointLocalPositions[0] = sbPos;
        pose.jointLocalRotations[0] = sbRot;

        // make first joint (hips) position and direction relative to the simulation bone
        var inverseSbRot = math.inverse(sbRot);
        pose.jointLocalPositions[1] = math.mul(inverseSbRot, frameRootMotion - sbPos);
        pose.jointLocalRotations[1] = math.mul(inverseSbRot, (quaternion)rotations[0]);
    }

    private static void ExtractPoseVelocities(ref PoseVector pose, in PoseVector nextPose, AnnotatedAnimationClip animationClip)
    {
        for (var jointIdx = 0; jointIdx < pose.jointLocalPositions.Length; jointIdx++)
        {
            var nextPos = nextPose.jointLocalPositions[jointIdx];
            var pos = pose.jointLocalPositions[jointIdx];
            pose.jointLocalVelocities[jointIdx] = (nextPos - pos) / animationClip.FrameTime;

            var nextRot = nextPose.jointLocalRotations[jointIdx];
            var rot = pose.jointLocalRotations[jointIdx];
            pose.jointLocalAngularVelocities[jointIdx] =
                MathExtensions.AngularVelocity(rot, nextRot, animationClip.FrameTime);
        }

        // root motion
        var vel = (nextPose.jointLocalPositions[0] - pose.jointLocalPositions[0]) / animationClip.FrameTime;

        // transform the root velocity so that it is
        // with respect to the root space of the current pose
        vel = math.mul(math.inverse(pose.jointLocalRotations[0]), vel);
        pose.jointLocalVelocities[0] = vel;
    }

    private static void ExtractPoseContacts(ref PoseVector pose, Skeleton skeleton, int leftToesIndex,
        int rightToesIndex, float contactVelocityThreshold)
    {
        // Contact with the ground when the joint is below a velocity threshold
        // TODO: Consider distance from the ground/contact when the joint is below a velocity threshold
        var leftToeVel = skeleton.GetWorldSpaceVelocity(skeleton.Joints[leftToesIndex], pose);
        var rightToeVel = skeleton.GetWorldSpaceVelocity(skeleton.Joints[rightToesIndex], pose);

        pose.leftFootContact = math.length(leftToeVel) < contactVelocityThreshold;
        pose.rightFootContact = math.length(rightToeVel) < contactVelocityThreshold;
    }
}
}