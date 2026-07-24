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
            for (var i = 0; i < nFrames; i++)
            {
                poses[i] = ExtractPose(bvhAnimation, i, mmData, poses);
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
                leftFootContact[i] = poses[i].LeftFootContact;
                rightFootContact[i] = poses[i].RightFootContact;
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
                pose.LeftFootContact = leftFootContactWindow[medianIndex];
                pose.RightFootContact = rightFootContactWindow[medianIndex];
                poses[i] = pose;
            }
        }

        // TODO: implement low-pass filter
        private static void SmoothSimulationBone(PoseVector[] poses, PoseSet poseSet)
        {
            // Save Hips world position and rotation before smoothing Simulation Bone (Root)
            var hipsWorldPositions = new Vector3[poses.Length];
            var hipsWorldRotations = new Quaternion[poses.Length];
            if (!poseSet.Skeleton.TryFind(HumanBodyBones.Hips, out var hipsJoint))
            {
                Debug.LogError("Hips Joint not found");
            }
            for (var i = 0; i < poses.Length; i++)
            {
                hipsWorldPositions[i] = poses[i].GetWorldSpacePosition(poseSet.Skeleton, hipsJoint);
                hipsWorldRotations[i] = poses[i].GetWorldSpaceRotation(poseSet.Skeleton, hipsJoint);
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
                sbPosX[i] = poses[i].JointLocalPositions[0].x;
                sbPosY[i] = poses[i].JointLocalPositions[0].y;
                sbPosZ[i] = poses[i].JointLocalPositions[0].z;
                var rot = poses[i].JointLocalRotations[0];
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
                poses[i].JointLocalPositions[0] = new float3(sbPosX[i], sbPosY[i], sbPosZ[i]);
                var dir = new float3(sbDirX[i], sbDirY[i], sbDirZ[i]);
                dir = math.normalize(dir);
                poses[i].JointLocalRotations[0] = math.normalize(quaternion.LookRotation(dir, math.up()));
            }
            // Set new relative Hips position and rotation to the Simulation Bone
            for (var i = 0; i < poses.Length; i++)
            {
                var sbPos = poses[i].JointLocalPositions[0];
                var inverseSbRot = math.inverse(poses[i].JointLocalRotations[0]);
                var newHipsPos = math.mul(inverseSbRot, hipsWorldPositions[i] - sbPos);
                var newHipsRot = math.mul(inverseSbRot, hipsWorldRotations[i]);
                poses[i].JointLocalPositions[1] = newHipsPos;
                poses[i].JointLocalRotations[1] = newHipsRot;
            }

            // Look up to the TODO 
            throw new NotImplementedException();
        }

        private static PoseVector ExtractPose(BvhAnimation bvhAnimation, int frameIndex, MotionMatchingData mmData, PoseVector[] poses)
        {
            var frame = bvhAnimation.Frames[frameIndex];

            var nJoints = bvhAnimation.Skeleton.Joints.Count;
            var outNJoint = nJoints + 1; // +1 for SimulationBone
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

            var jointLocalPositions = new Vector3[outNJoint];
            var jointLocalRotations = new Quaternion[outNJoint];
            var jointLocalVelocities = new Vector3[outNJoint];
            var jointLocalAngularVelocities = new Vector3[outNJoint];

            // Joints
            for (var i = 0; i < nJoints; i++)
            {
                jointLocalPositions[i + 1] = bvhAnimation.Skeleton.Joints[i].localOffset;
                jointLocalRotations[i + 1] = frame.localRotations[i];
            }

            // SimulationBone
            // position and direction are hips projected on the ground
            float3 frameRootMotion = frame.rootMotion;
            var sbPos = new float3(frameRootMotion.x, 0.0f, frameRootMotion.z);
            var hipsForwardDir = math.mul(frame.localRotations[0], mmData.HipsForwardLocalVector);
            hipsForwardDir.y = 0;
            hipsForwardDir = math.normalize(hipsForwardDir);
            var sbRot = quaternion.LookRotation(hipsForwardDir, math.up());
            jointLocalPositions[0] = sbPos;
            jointLocalRotations[0] = sbRot;

            // make first joint (hips) position and direction relative to the simulation bone
            var inverseSbRot = math.inverse(sbRot);
            jointLocalPositions[1] = math.mul(inverseSbRot, frameRootMotion - sbPos);
            jointLocalRotations[1] = math.mul(inverseSbRot, frame.localRotations[0]);

            if (frameIndex == 0)
            {
                for (var i = 0; i < jointLocalVelocities.Length; i++) jointLocalVelocities[i] = float3.zero;
                for (var i = 0; i < jointLocalAngularVelocities.Length; i++) jointLocalAngularVelocities[i] = float3.zero;
            }
            else
            {
                var prevPos = poses[frameIndex - 1];

                for (var joint = 0; joint < outNJoint; joint++)
                {
                    var pos = jointLocalPositions[joint];
                    var prevLocalPosition = prevPos.JointLocalPositions[joint];
                    jointLocalVelocities[joint] = (pos - prevLocalPosition) / bvhAnimation.FrameTime;

                    var rot = jointLocalRotations[joint];
                    var prevLocalRotation = prevPos.JointLocalRotations[joint];
                    jointLocalAngularVelocities[joint] = MathExtensions.AngularVelocity(prevLocalRotation, rot, bvhAnimation.FrameTime);
                }
            }

            // Compute Contact
            // Contact with the ground when the joint is below a velocity threshold
            ForwardKinematics(bvhAnimation.Skeleton,
                              jointLocalPositions, jointLocalRotations, jointLocalVelocities, jointLocalAngularVelocities,
                              out _, out _, out var jointVelocities, out _);
            var contactLeftFoot = math.length(jointVelocities[leftToesIndex]) < mmData.ContactVelocityThreshold;
            var contactRightFoot = math.length(jointVelocities[rightToesIndex]) < mmData.ContactVelocityThreshold;

            // Result
            return new PoseVector(jointLocalPositions, jointLocalRotations,
                                  jointLocalVelocities, jointLocalAngularVelocities,
                                  contactLeftFoot, contactRightFoot);
        }

        private static void ForwardKinematics(Skeleton skeleton,
                                       Vector3[] jointLocalPositions, Quaternion[] jointLocalRotations, Vector3[] jointLocalVelocities, Vector3[] jointLocalAngularVelocities,
                                       out Vector3[] jointPositions, out Quaternion[] jointRotations, out Vector3[] jointVelocities, out Vector3[] jointAngularVelocities)
        {
            jointPositions = new Vector3[jointLocalPositions.Length];
            jointRotations = new Quaternion[jointLocalRotations.Length];
            jointVelocities = new Vector3[jointLocalVelocities.Length];
            jointAngularVelocities = new Vector3[jointLocalAngularVelocities.Length];
            jointPositions[0] = jointLocalPositions[0];
            jointRotations[0] = jointLocalRotations[0];
            jointVelocities[0] = jointLocalVelocities[0];
            jointAngularVelocities[0] = jointLocalAngularVelocities[0];
            for (var j = 1; j < skeleton.Joints.Count + 1; j++) // +1 for SimulationBone
            {
                var joint = skeleton.Joints[j - 1];
                var parentIndex = 0;
                if (j > 1) parentIndex = joint.parentIndex + 1; // +1 for SimulationBone
                var rotatedLocalOffset = jointRotations[parentIndex] * jointLocalPositions[j];
                jointPositions[j] = rotatedLocalOffset + jointPositions[parentIndex];
                jointRotations[j] = jointRotations[parentIndex] * jointLocalRotations[j];
                // Given a fixed point 'O', a point 'A' relative to 'O', and the angular velocity 'w' of 'O'
                // the velocity 'V' of 'A' is 'V = w x OA' where 'x' is the cross product and 'OA' is the vector from 'O' to 'A'
                // Here, we add the local velocity + the velocity caused by the angular velocity + parent velocity
                jointVelocities[j] = jointRotations[parentIndex] * jointLocalVelocities[j] +
                                     Vector3.Cross(jointAngularVelocities[parentIndex], rotatedLocalOffset) +
                                     jointVelocities[parentIndex];
                jointAngularVelocities[j] = jointRotations[parentIndex] * jointLocalAngularVelocities[j] +
                                            jointAngularVelocities[parentIndex];
            }
        }

    }
}