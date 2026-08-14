using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace AnimationTools
{
/// <summary>
/// Everything the pose-database pipeline needs from the asset it is extracting.
///
/// <see cref="PoseSet"/>, <see cref="PoseExtractor"/> and <see cref="PoseSerializer"/> used to take
/// a <see cref="MotionMatchingData"/> directly, which meant anything wanting a pose database also
/// had to be a full Motion Matching asset -- trajectory features, pose features, T-Pose, mecanim
/// map and all. Only a handful of those members actually reach the pose path, and this interface is
/// exactly that handful, so a consumer such as MotionField's config can supply a database without
/// dragging in the feature-set machinery it never uses.
///
/// This lives in the AnimationTools runtime assembly since it is generic pose-database
/// infrastructure, not specific to any one motion-synthesis technique.
/// </summary>
public interface IPoseSetSource
{
    /// <summary>Asset name. Doubles as the database folder and file base name.</summary>
    string name { get; }

    /// <summary>Local axis of the hips pointing forward, used to orient the simulation bone.</summary>
    float3 HipsForwardLocalVector { get; }

    /// <summary>Foot speed below which a toe counts as planted.</summary>
    float ContactVelocityThreshold { get; }

    /// <summary>Maps animation channel names onto Mecanim bones. Pose extraction needs the toes.</summary>
    IReadOnlyList<JointToMecanim> AnimationChannelToMecanim { get; }

    /// <summary>
    /// Maps an animation channel name onto a Mecanim bone. Pose extraction needs this to locate the
    /// toes for contact detection.
    /// </summary>
    bool TryGetMecanimBone(string jointName, out HumanBodyBones bone);

    /// <summary>Directory the serialized database lives in. Created if missing (editor only).</summary>
    string GetAssetPath();
}
}
