#ifndef OPENPOSE_CORE_ENUM_CLASSES_HPP
#define OPENPOSE_CORE_ENUM_CLASSES_HPP

namespace pose
{
    enum class ScaleMode : unsigned char
    {
        InputResolution,
        NetOutputResolution,
        OutputResolution,
        ZeroToOne, // [0, 1]
        PlusMinusOne, // [-1, 1]
        UnsignedChar, // [0, 255]
    };

    enum class HeatMapType : unsigned char
    {
        Parts,
        Background,
        PAFs,
    };

    enum class RenderMode : unsigned char
    {
        None,
        Cpu,
        Gpu,
    };
}

#endif // OPENPOSE_CORE_ENUM_CLASSES_HPP
