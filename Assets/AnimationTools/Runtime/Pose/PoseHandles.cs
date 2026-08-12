namespace AnimationTools
{
// Six structurally identical handle types kept distinct on purpose: the type system
// stops a PositionHandle from being used where a VelocityHandle is expected, even though
// both are just an element index into a PoseBuffer section.

public readonly struct PositionHandle
{
    internal readonly int Index;
    internal PositionHandle(int index) { Index = index; }
    public bool IsValid => Index >= 0;
    public static PositionHandle Invalid => new(-1);
}

public readonly struct RotationHandle
{
    internal readonly int Index;
    internal RotationHandle(int index) { Index = index; }
    public bool IsValid => Index >= 0;
    public static RotationHandle Invalid => new(-1);
}

public readonly struct ScaleHandle
{
    internal readonly int Index;
    internal ScaleHandle(int index) { Index = index; }
    public bool IsValid => Index >= 0;
    public static ScaleHandle Invalid => new(-1);
}

public readonly struct BoolHandle
{
    internal readonly int Index;
    internal BoolHandle(int index) { Index = index; }
    public bool IsValid => Index >= 0;
    public static BoolHandle Invalid => new(-1);
}

public readonly struct VelocityHandle
{
    internal readonly int Index;
    internal VelocityHandle(int index) { Index = index; }
    public bool IsValid => Index >= 0;
    public static VelocityHandle Invalid => new(-1);
}

public readonly struct AngularVelocityHandle
{
    internal readonly int Index;
    internal AngularVelocityHandle(int index) { Index = index; }
    public bool IsValid => Index >= 0;
    public static AngularVelocityHandle Invalid => new(-1);
}
}
