#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_UTIL_H_
#define ANDROIDCARDUINO_AUTODRIVE_UTIL_H_

#include <opencv2/core/core.hpp>
#include <memory>

//namespace Autodrive
//{
    using POINT = cv::Point2f;
    using UINT = unsigned int;

    struct Mathf
    {
        static const float PI;
        static const float PI_2;
        static int toDegrees(float radians) { return int(radians * 180.f / PI); }
    };

    struct Direction
    {
        static const float RIGHT;
        static const float LEFT;
        static const float FORWARD;
    };
   
    #ifndef __ANDROID__
    void show_image(cv::Mat mat, int resize, std::string wName);
    #endif
    
    template<typename T, typename... Args>
    std::unique_ptr<T> make_unique(Args&&... args) 
    {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }

    template<class TYPE>
    struct optional
    {
        optional(TYPE value) : valid(true), val(value)
        {
        }
        optional() : valid(false)
        {
        }
        optional(::std::nullptr_t) : valid(false)
        {
        }
        explicit operator bool() const
        {
            return valid;
        }
        TYPE* operator->()
        {
            return &val;
        }
        TYPE operator* () const
        {
            return val;
        }
        bool valid;
    private:
        TYPE val;
    };

    struct SearchResult
    {
        POINT point;
        int distance;
        bool found = false;
    };

    SearchResult firstnonzero_direction(const cv::Mat& mat, cv::Point_ < float > start, float direction, int maxDist);
	
    optional<cv::Point> firstnonzero_horizontal(const cv::Mat& mat, cv::Point iterator)
    {
        while (iterator.x < mat.size().width - 1)
        {
            if (mat.at<uchar>(iterator) != 0)
            {
                return iterator;
            }
            iterator.x++;
        }
        return nullptr;
    }

    template<class numeric_t>
    numeric_t weighted_average(numeric_t val1, numeric_t val2, numeric_t val1_multiplier);
//}

#endif //ANDROIDCARDUINO_AUTODRIVE_UTIL_H_