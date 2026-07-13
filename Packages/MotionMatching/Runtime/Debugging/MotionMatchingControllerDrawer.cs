using MotionMatching.Editor;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
/// <summary>
/// To be added alongside a MotionMatchingController to draw debug information in the viewport.
/// </summary>
public class MotionMatchingControllerDrawer : MonoBehaviour
{
    [Header("Debug")]
    
    public float spheresRadius = 0.1f;
    public bool debugSkeleton = true;
    public bool debugFutureSkeleton = true;
    public bool debugCurrent = true;
    public bool debugPose = true;
    public bool debugTrajectory = true;
    public bool debugEnvironment = true;
    public bool debugContacts = true;

    public void OnDrawGizmos()
    {
        if(!isActiveAndEnabled) return;
        if (TryGetComponent(out MotionMatchingController mmc))
        {
            DrawGizmoForMotionMatchingController(mmc);
        }
    }
    
    
    private void DrawGizmoForMotionMatchingController(MotionMatchingController mmc)
    {
        if (mmc.SkeletonTransforms == null || mmc.PoseSet == null) return;

        if (debugSkeleton)
        {
            Gizmos.color = Color.red;
            for (int i = 2; i < mmc.SkeletonTransforms.Length; i++) // skip Simulation Bone
            {
                Transform t = mmc.SkeletonTransforms[i];
                GizmosExtensions.DrawLine(t.parent.position, t.position, 6.0f);
            }
        }

        // Character
        if (mmc.PoseSet == null) return;

        int currentFrame = mmc.CurrentFrame;
        float3 characterOrigin = mmc.SkeletonTransforms[0].position;
        float3 characterForward = mmc.SkeletonTransforms[0].forward;

        if (debugFutureSkeleton)
        {
            // Find Main Position Trajectory
            MotionMatchingData.TrajectoryFeature feature = null;
            for (int i = 0; i < mmc.mmData.TrajectoryFeatures.Count; i++)
            {
                if (mmc.mmData.TrajectoryFeatures[i].IsMainPositionFeature)
                {
                    feature = mmc.mmData.TrajectoryFeatures[i];
                }
            }

            if (feature != null)
            {
                for (int p = 0; p < feature.FramesPrediction.Length; p++)
                {
                    Gizmos.color = Color.red + Color.cyan * ((float)p / feature.FramesPrediction.Length);
                    int frame = currentFrame + feature.FramesPrediction[p];
                    mmc.PoseSet.GetPose(frame, out PoseVector futurePose);
                    var simulationBoneTransform = mmc.GetSimulationBoneWorldSpaceTransform(futurePose);
                    var worldPositions = mmc.PoseSet.GetWorldPositions(futurePose, simulationBoneTransform);
                    for (int i = 2; i < mmc.PoseSet.Skeleton.Joints.Count; i++)
                    {
                        float3 child = worldPositions[i];
                        float3 parent = worldPositions[mmc.PoseSet.Skeleton.Joints[i].parentIndex];
                        GizmosExtensions.DrawLine(parent, child, 3);
                    }
                }
            }
        }

        if (debugCurrent)
        {
            Gizmos.color = new Color(1.0f, 0.0f, 0.5f, 1.0f);
            Gizmos.DrawSphere(characterOrigin, spheresRadius);
            GizmosExtensions.DrawArrow(characterOrigin, characterOrigin + characterForward * 1.5f, thickness: 4);
        }

        if (debugContacts)
        {
            Gizmos.color = Color.green;
            if (mmc.IsLeftFootContact)
            {
                Gizmos.DrawSphere(mmc.SkeletonTransforms[mmc.LeftToesIndex].position, spheresRadius);
            }

            if (mmc.IsRightFootContact)
            {
                Gizmos.DrawSphere(mmc.SkeletonTransforms[mmc.RightToesIndex].position, spheresRadius);
            }
        }

        // Feature Set
        if (mmc.FeatureSet == null) return;

        MotionMatchingDataVisualiser.DrawFeatureGizmos(mmc.FeatureSet, mmc.mmData, spheresRadius, currentFrame,
            characterOrigin, characterForward,
            mmc.SkeletonTransforms, mmc.PoseSet.Skeleton, Color.blue, debugPose, debugTrajectory,
            debugEnvironment);

    }
}
}