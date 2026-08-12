namespace AnimationTools
{
/// <summary>Kind of data a channel stores.</summary>
/// <remarks>
/// <see cref="Velocity"/> and <see cref="AngularVelocity"/> are per-second rates, not
/// per-frame deltas; scale by frame time to get a one-frame delta.
/// </remarks>
public enum ChannelType : byte { Position, Rotation, Scale, Bool, Velocity, AngularVelocity }

/// <summary>Reference frame a channel's values are expressed in.</summary>
public enum ChannelSpace : byte { ParentLocal, RootLocal, Character, World }

/// <summary>
/// On-disk/in-buffer encoding of a rotation-valued channel.
/// <see cref="RotationRepresentation.RotationVector"/> is axis * radians; angular velocity
/// channels always use this representation (axis * radians/second) since a quaternion has
/// no meaningful "rate" form.
/// </summary>
public enum RotationRepresentation : byte { Quaternion, RotationVector, EulerDegrees }

/// <summary>
/// Disambiguates multiple channels of the same <see cref="ChannelType"/> on the same bone.
/// In particular, root motion is an explicit, discoverable channel declaration
/// (<see cref="RootMotion"/>) rather than an implicit "index 0" convention.
/// </summary>
public enum ChannelUsage : byte { Default, RootMotion, Contact }
}
