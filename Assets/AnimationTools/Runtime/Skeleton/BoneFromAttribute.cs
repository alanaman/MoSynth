using System;
using UnityEngine;

namespace AnimationTools
{
/// <summary>
/// Marks a bone-reference field as bound to a particular skeleton, so a custom property drawer
/// can show a bone-name dropdown instead of a raw name field.
/// </summary>
/// <remarks>
/// <see cref="SkeletonMemberName"/> names a sibling serialized field or property on the
/// same object that holds the skeleton the bone dropdown filters against.
/// </remarks>
[AttributeUsage(AttributeTargets.Field)]
public sealed class BoneFromAttribute : PropertyAttribute
{
    public string SkeletonMemberName { get; }

    public BoneFromAttribute(string skeletonMemberName)
    {
        SkeletonMemberName = skeletonMemberName;
    }
}
}
