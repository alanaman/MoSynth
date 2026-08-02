using UnityEngine;
using Unity.Mathematics;
using System;

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
    public static bool Extract(AnimationData animation, PoseSet poseSet, MotionMatchingData mmData)
    {
        var bvhAnimation = animation.GetAnimation();
        // Set Poses
        var nFrames = bvhAnimation.Frames.Length;
        var poses = new PoseVector[nFrames];
        var nBvhJoints = bvhAnimation.Skeleton.Joints.Count;
        var nPoseSetJoints = nBvhJoints + 1; // +1 for SimulationBone
        if (!bvhAnimation.Skeleton.TryFind(HumanBodyBones.LeftToes, out var leftToesJoint))
        {
            Debug.LogError("LeftToes not found in BVHAnimation");
        }

        var leftToesIndex = leftToesJoint.index + 1; // +1 for SimulationBone
        if (!bvhAnimation.Skeleton.TryFind(HumanBodyBones.RightToes, out var rightToesJoint))
        {
            Debug.LogError("RightToes not found in BVHAnimation");
        }

        var rightToesIndex = rightToesJoint.index + 1; // +1 for SimulationBone

        for (var i = 0; i < nFrames; i++)
        {
            poses[i] = new PoseVector(nPoseSetJoints);
            ExtractPose(ref poses[i], bvhAnimation, i, mmData);
        }

        for (var i = 0; i < nFrames - 1; i++)
        {
            ExtractPoseVelocities(ref poses[i], poses[i + 1], bvhAnimation);
        }

        for (var i = 0; i < nFrames; i++)
        {
            // Note: this requires velocities to be pre-calculated
            ExtractPoseContacts(ref poses[i],
                bvhAnimation.Skeleton,
                leftToesIndex,
                rightToesIndex,
                mmData.ContactVelocityThreshold);
        }


        SmoothContacts(poses);

        if (poseSet.AddClip(poses, bvhAnimation.FrameTime, animation.Tags))
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

    // TODO: implement low-pass filter
    private static void SmoothSimulationBone(PoseVector[] poses, PoseSet poseSet)
    {
        // Save Hips world position and rotation before smoothing Simulation Bone (Root)
        var hipsWorldPositions = new float3[poses.Length];
        var hipsWorldRotations = new quaternion[poses.Length];
        if (!poseSet.Skeleton.TryFind(HumanBodyBones.Hips, out var hipsJoint))
        {
            Debug.LogError("Hips Joint not found");
        }

        for (var i = 0; i < poses.Length; i++)
        {
            hipsWorldPositions[i] = poseSet.Skeleton.GetWorldSpacePosition(hipsJoint, poses[i]);
            hipsWorldRotations[i] = poseSet.Skeleton.GetWorldSpaceRotation(hipsJoint, poses[i]);
        }

        // Prepare simulation bone lists for python
        var sbPosX = new float[poses.Length];
        var sbPosY = new float[poses.Length];
        var sbPosZ = new float[poses.Length];
        var sbDirX = new float[poses.Length];
        var sbDirY = new float[poses.Length];
        var sbDirZ = new float[poses.Length];
        for (var i = 0; i < poses.Length; i++)
        {
            sbPosX[i] = poses[i].jointLocalPositions[0].x;
            sbPosY[i] = poses[i].jointLocalPositions[0].y;
            sbPosZ[i] = poses[i].jointLocalPositions[0].z;
            var rot = poses[i].jointLocalRotations[0];
            var dir = math.mul(rot, new float3(0, 0, 1));
            sbDirX[i] = dir.x;
            sbDirY[i] = dir.y;
            sbDirZ[i] = dir.z;
        }

        // TODO: Implement Low-Pass filter here...
        //       the previous arrays (sbPos/Dir*) where used for a previous implementation
        //       with python... consider change them

        // Set new Simulation Bone positions and rotations
        for (var i = 0; i < poses.Length; i++)
        {
            poses[i].jointLocalPositions[0] = new float3(sbPosX[i], sbPosY[i], sbPosZ[i]);
            var dir = new float3(sbDirX[i], sbDirY[i], sbDirZ[i]);
            dir = math.normalize(dir);
            poses[i].jointLocalRotations[0] = math.normalize(quaternion.LookRotation(dir, math.up()));
        }

        // Set new relative Hips position and rotation to the Simulation Bone
        for (var i = 0; i < poses.Length; i++)
        {
            var sbPos = poses[i].jointLocalPositions[0];
            var inverseSbRot = math.inverse(poses[i].jointLocalRotations[0]);
            var newHipsPos = math.mul(inverseSbRot, hipsWorldPositions[i] - sbPos);
            var newHipsRot = math.mul(inverseSbRot, hipsWorldRotations[i]);
            poses[i].jointLocalPositions[1] = newHipsPos;
            poses[i].jointLocalRotations[1] = newHipsRot;
        }

        // Look up to the TODO 
        throw new NotImplementedException();
    }

    private static void ExtractPose(ref PoseVector pose, BvhAnimation bvhAnimation, int frameIndex,
        MotionMatchingData mmData)
    {
        var frame = bvhAnimation.Frames[frameIndex];
        // Joints
        for (var i = 1; i < pose.jointLocalPositions.Length; i++)
        {
            pose.jointLocalPositions[i] = bvhAnimation.Skeleton.Joints[i - 1].localOffset;
            pose.jointLocalRotations[i] = frame.localRotations[i - 1];
        }

        // SimulationBone
        // position and direction are hips projected on the ground
        var frameRootMotion = frame.rootMotion;
        var sbPos = new Vector3(frameRootMotion.x, 0.0f, frameRootMotion.z);
        var hipsForwardDir = frame.localRotations[0] * mmData.HipsForwardLocalVector;
        hipsForwardDir.y = 0;
        hipsForwardDir = hipsForwardDir.normalized;
        var sbRot = Quaternion.LookRotation(hipsForwardDir, Vector3.up);
        pose.jointLocalPositions[0] = sbPos;
        pose.jointLocalRotations[0] = sbRot;

        // make first joint (hips) position and direction relative to the simulation bone
        var inverseSbRot = math.inverse(sbRot);
        pose.jointLocalPositions[1] = math.mul(inverseSbRot, frameRootMotion - sbPos);
        pose.jointLocalRotations[1] = math.mul(inverseSbRot, frame.localRotations[0]);
    }

    private static void ExtractPoseVelocities(ref PoseVector pose, in PoseVector nextPose, BvhAnimation bvhAnimation)
    {
        for (var jointIdx = 0; jointIdx < pose.jointLocalPositions.Length; jointIdx++)
        {
            var nextPos = nextPose.jointLocalPositions[jointIdx];
            var pos = pose.jointLocalPositions[jointIdx];
            pose.jointLocalVelocities[jointIdx] = (nextPos - pos) / bvhAnimation.FrameTime;

            var nextRot = nextPose.jointLocalRotations[jointIdx];
            var rot = pose.jointLocalRotations[jointIdx];
            pose.jointLocalAngularVelocities[jointIdx] =
                MathExtensions.AngularVelocity(rot, nextRot, bvhAnimation.FrameTime);
        }
        
        // root motion
        var vel = (nextPose.jointLocalPositions[0] - pose.jointLocalPositions[0]) / bvhAnimation.FrameTime;

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

    private static void ForwardKinematics(Skeleton skeleton,
        float3[] jointLocalPositions, quaternion[] jointLocalRotations, float3[] jointLocalVelocities,
        float3[] jointLocalAngularVelocities,
        out float3[] jointPositions, out quaternion[] jointRotations, out float3[] jointVelocities,
        out float3[] jointAngularVelocities)
    {
        jointPositions = new float3[jointLocalPositions.Length];
        jointRotations = new quaternion[jointLocalRotations.Length];
        jointVelocities = new float3[jointLocalVelocities.Length];
        jointAngularVelocities = new float3[jointLocalAngularVelocities.Length];
        jointPositions[0] = jointLocalPositions[0];
        jointRotations[0] = jointLocalRotations[0];
        jointVelocities[0] = jointLocalVelocities[0];
        jointAngularVelocities[0] = jointLocalAngularVelocities[0];
        for (var j = 1; j < skeleton.Joints.Count + 1; j++) // +1 for SimulationBone
        {
            var joint = skeleton.Joints[j - 1];
            var parentIndex = 0;
            if (j > 1) parentIndex = joint.parentIndex + 1; // +1 for SimulationBone
            var rotatedLocalOffset = math.mul(jointRotations[parentIndex], jointLocalPositions[j]);
            jointPositions[j] = rotatedLocalOffset + jointPositions[parentIndex];
            jointRotations[j] = math.mul(jointRotations[parentIndex], jointLocalRotations[j]);
            // Given a fixed point 'O', a point 'A' relative to 'O', and the angular velocity 'w' of 'O'
            // the velocity 'V' of 'A' is 'V = w x OA' where 'x' is the cross product and 'OA' is the vector from 'O' to 'A'
            // Here, we add the local velocity + the velocity caused by the angular velocity + parent velocity
            jointVelocities[j] =
                math.mul(jointRotations[parentIndex], jointLocalVelocities[j]) +
                math.cross(jointAngularVelocities[parentIndex], rotatedLocalOffset) +
                jointVelocities[parentIndex];
            jointAngularVelocities[j] =
                math.mul(jointRotations[parentIndex], jointLocalAngularVelocities[j]) +
                jointAngularVelocities[parentIndex];
        }
    }
}
}