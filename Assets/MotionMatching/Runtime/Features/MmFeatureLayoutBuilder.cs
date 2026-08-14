using System;
using System.Collections.Generic;
using AnimationTools;
using UnityEngine;

namespace MotionMatching
{
public class MmFeatureLayoutBuilder
{
    List<ChannelDescriptor> GetChannelDescriptors(MotionMatchingData mmData)
    {
        var poseSet = mmData.GetOrImportPoseSet();
        var numberFeatureVectors = poseSet.NumberPoses;
        var numTrajectoryFeatures = mmData.trajectoryFeatures.Count;
        var numPoseFeatures = mmData.poseFeatures.Count;

        var channelDescriptors = new List<ChannelDescriptor>();

        // Trajectory feature
        foreach (var trajectoryFeature in mmData.trajectoryFeatures)
        {
            if (trajectoryFeature.simulationBone) continue;
            switch (trajectoryFeature.featureType)
            {
                case MotionMatchingData.TrajectoryFeature.Type.Position:
                    channelDescriptors.Add(
                        new ChannelDescriptor()
                        {
                            boneId = poseSet.SkeletonAsset.FindJointIndexOrZero(trajectoryFeature.bone),
                            space = ChannelSpace.RootLocal,
                            type = ChannelType.Position,
                        });
                    break;
                case MotionMatchingData.TrajectoryFeature.Type.Direction:
                    channelDescriptors.Add(
                        new ChannelDescriptor()
                        {
                            boneId = poseSet.SkeletonAsset.FindJointIndexOrZero(trajectoryFeature.bone),
                            space = ChannelSpace.RootLocal,
                            type = ChannelType.Direction,
                        }
                    );
                    break;
                default:
                    Debug.Assert(false, "Unknown PoseFeature.Type: " + trajectoryFeature.featureType);
                    break;
            }
        }

        // Pose features
        for (var i = 0; i < numPoseFeatures; i++)
        {
            var poseFeature = mmData.poseFeatures[i];
            switch (poseFeature.featureType)
            {
                case MotionMatchingData.PoseFeature.Type.Position:
                    channelDescriptors.Add(
                        new ChannelDescriptor()
                        {
                            boneId = poseSet.SkeletonAsset
                                .FindJointIndexOrZero(poseFeature.bone),
                            space = ChannelSpace.RootLocal,
                            type = ChannelType.Position,
                        });
                    break;
                case MotionMatchingData.PoseFeature.Type.Velocity:
                    channelDescriptors.Add(
                        new ChannelDescriptor()
                        {
                            boneId = poseSet.SkeletonAsset
                                .FindJointIndexOrZero(poseFeature.bone),
                            space = ChannelSpace.RootLocal,
                            type = ChannelType.Velocity,
                        });
                    break;
                default:
                    Debug.Assert(false, "Unknown PoseFeature.Type: " + poseFeature.featureType);
                    break;
            }
        }
        
        return channelDescriptors;
    }

    PoseLayout GetFeatureLayout(MotionMatchingData mmData)
    {
        var channels = GetChannelDescriptors(mmData);
        var layout = PoseLayout.Build(mmData.GetOrImportPoseSet().SkeletonAsset,  channels);
        return layout;
    }
}
}