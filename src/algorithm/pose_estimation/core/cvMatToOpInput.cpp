#include <algorithm/ulspose/utilities/errorAndLog.hpp>
#include <algorithm/ulspose/utilities/fastMath.hpp>
#include <algorithm/ulspose/utilities/openCv.hpp>
#include <algorithm/ulspose/core/cvMatToOpInput.hpp>

namespace pose
{
    CvMatToOpInput::CvMatToOpInput(const Point<int>& netInputResolution, const int scaleNumber, const float scaleGap) :
        mScaleNumber{scaleNumber},
        mScaleGap{scaleGap},
        mInputNetSize4D{{mScaleNumber, 3, netInputResolution.y, netInputResolution.x}}
    {
        try
        {
            // Security checks
            if (netInputResolution.x % 8 != 0 || netInputResolution.y % 8 != 0)
                error("Net input resolution must be multiples of 8.", __LINE__, __FUNCTION__, __FILE__);
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    std::pair<Array<float>, std::vector<float>> CvMatToOpInput::format(const cv::Mat& cvInputData) const
    {
        try
        {
            // Security checks
            if (cvInputData.empty())
                error("Wrong input element (empty cvInputData).", __LINE__, __FUNCTION__, __FILE__);

            // inputNetData - Reescale keeping aspect ratio and transform to float the input deep net image
            Array<float> inputNetData{mInputNetSize4D};
            std::vector<float> scaleRatios(mScaleNumber, 1.f);
            const auto inputNetDataOffset = inputNetData.getVolume(1, 3);
            for (auto i = 0; i < mScaleNumber; i++)
            {
                const auto currentScale = 1.f - i*mScaleGap;
                if (currentScale < 0.f || 1.f < currentScale)
                    error("All scales must be in the range [0, 1], i.e. 0 <= 1-scale_number*scale_gap <= 1", __LINE__, __FUNCTION__, __FILE__);

                const auto netInputWidth = inputNetData.getSize(3);
                const auto targetWidth  = fastTruncate(intRound(netInputWidth * currentScale) / 8 * 8, 1, netInputWidth);
                const auto netInputHeight = inputNetData.getSize(2);
                const auto targetHeight  = fastTruncate(intRound(netInputHeight * currentScale) / 8 * 8, 1, netInputHeight);
                const Point<int> targetSize{targetWidth, targetHeight};
                const auto scale = resizeGetScaleFactor(Point<int>{cvInputData.cols, cvInputData.rows}, targetSize);
                const cv::Mat frameWithNetSize = resizeFixedAspectRatio(cvInputData, scale, Point<int>{netInputWidth, netInputHeight});
                // Fill inputNetData
                uCharCvMatToFloatPtr(inputNetData.getPtr() + i * inputNetDataOffset, frameWithNetSize, true);
                // Fill scaleRatios
                scaleRatios[i] = {(float)scale};
                if (i > 0)
                    scaleRatios[i] /= scaleRatios[0];
            }
            scaleRatios.at(0) /= scaleRatios[0];
            return std::make_pair(inputNetData, scaleRatios);
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return std::make_pair(Array<float>{}, std::vector<float>{});
        }
    }
}
