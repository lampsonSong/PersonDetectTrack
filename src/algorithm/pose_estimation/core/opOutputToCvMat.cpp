#include <algorithm/ulspose/utilities/errorAndLog.hpp>
#include <algorithm/ulspose/utilities/openCv.hpp>
#include <algorithm/ulspose/core/opOutputToCvMat.hpp>

namespace pose
{
    OpOutputToCvMat::OpOutputToCvMat(const Point<int>& outputResolution) :
        mOutputResolution{outputResolution}
    {
    }

    cv::Mat OpOutputToCvMat::formatToCvMat(const Array<float>& outputData) const
    {
        try
        {
            // Security checks
            if (outputData.empty())
                error("Wrong input element (empty outputData).", __LINE__, __FUNCTION__, __FILE__);

            cv::Mat cvMat;
            floatPtrToUCharCvMat(cvMat, outputData.getConstPtr(), mOutputResolution, 3);

            return cvMat;
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return cv::Mat{};
        }
    }
}
