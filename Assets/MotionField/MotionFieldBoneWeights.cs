using Python.Runtime;

namespace MotionField
{
/// <summary>
/// Marshals <see cref="MotionFieldConfig.boneWeights"/> into the mapping Python resolves against
/// the skeleton.
/// </summary>
/// <remarks>
/// Shared by the runtime stage and the two Editor buttons, because a value function trained under
/// one weight table and run under another is exactly what the staleness gate exists to catch --
/// and it can only catch it if both sides send the same thing.
/// </remarks>
public static class MotionFieldBoneWeights
{
    /// <summary>
    /// Build the <c>{joint name: (position, velocity)}</c> dict for
    /// <c>MotionField.resolve_bone_weights</c>. Must be called with the GIL held.
    /// </summary>
    /// <remarks>
    /// Always returns a dict, even an empty one. Python normalises an all-neutral table back to
    /// "uniform", so there is no need to decide here whether the weights are worth sending, and no
    /// second code path that could disagree about it.
    /// </remarks>
    public static PyDict ToPython(MotionFieldConfig config)
    {
        var dict = new PyDict();
        if (config == null) return dict;

        foreach (MotionFieldConfig.BoneWeight weight in config.boneWeights)
        {
            if (string.IsNullOrEmpty(weight.name) || weight.IsNeutral) continue;

            // The dict takes its own reference, so disposing the tuple here is safe.
            using var pair = new PyTuple(new[]
            {
                ((double)weight.position).ToPython(),
                ((double)weight.velocity).ToPython(),
            });
            dict[weight.name] = pair;
        }

        return dict;
    }
}
}
