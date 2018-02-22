#pragma once
#include "line.hpp"
#include "util.hpp"
#include "../settings.hpp"
#include "roadline.hpp"

namespace Autodrive
{
    class roadlinebuilder
    {
        static const int pointDist = 4;
        static const int maxDistFromStart = 22;
        static const int maxUpwardsIteration = 100;
        int totalGap = 0;
        int carY = 0;

        static SearchResult FindPoint(const cv::Mat& cannied, POINT start, float leftAngle, float rightAngle,float iterationReduction = 0)
        {
            SearchResult rightSearch = firstnonzero_direction(cannied, start, rightAngle, Settings::rightIterationLength - iterationReduction);
            SearchResult leftSearch = firstnonzero_direction(cannied, start, leftAngle, Settings::leftIterationLength -iterationReduction);
            if (leftSearch.found && rightSearch.found)
            {
                if (leftSearch.distance <= rightSearch.distance + 15)
                {
                    {
                        return leftSearch;
                    }
                } else
                {
                    return rightSearch;
                }
            } else if (leftSearch.found)
            {
                return leftSearch;
            }

            return rightSearch;
        }


        POINT GetFirstPoint(const cv::Mat& cannied)
        {
            SearchResult searchRes;
            int unfound = 0;

            POINT start_point = last_start;

            //SEARCH UPWARDS UNTIL HIT
            while (!searchRes.found && unfound++ < Settings::firstFragmentMaxDist)
            {
                searchRes = FindPoint(cannied, start_point, Direction::LEFT, Direction::RIGHT,Settings::iterateReduceOnStart);
                if (!searchRes.found)
                    start_point.y--;
            }

            // SEARCH DOWNWARDS IF HIT ON FIRST Y AXIS
            SearchResult new_hit;
            new_hit = searchRes;
            int yStart = 0;
            while (new_hit.found && unfound == 1 && yStart++ < 5 && searchRes.point.y < carY)
            {
                new_hit.point.y++;
                new_hit = FindPoint(cannied, new_hit.point, Direction::LEFT, Direction::RIGHT);
                if (new_hit.found && int(new_hit.point.x) == int(searchRes.point.x))
                {
                    searchRes = new_hit;
                } else
                    break;
            }

            if (linef(first_start, searchRes.point).length2() <= maxDistFromStart*maxDistFromStart)
                last_start = searchRes.point;

            return searchRes.point;
        }


        optional<POINT> GetNextPoint(const cv::Mat& cannied, float est_angle, const POINT& prevPoint,int delta = 2)
        {

            POINT it = prevPoint;
            SearchResult searchResult;
            int unfound = 0;
            while (!searchResult.found && unfound < maxUpwardsIteration && it.y > 0)
            {
                it.y-=delta;
                it.x += cosf(est_angle)*delta;
                searchResult = FindPoint(cannied, it, Direction::LEFT, Direction::RIGHT);
                totalGap++;
                unfound++;
            }

            if (searchResult.found)
                return searchResult.point;
            else
                return nullptr;
        }

    public:
        POINT first_start;
        POINT last_start;
        float centerX;

        roadlinebuilder(POINT startPoint, float center_x,int car_y) :
            carY(car_y),first_start(startPoint), last_start(startPoint), centerX(center_x)
        {

        }

        RoadLine build(const cv::Mat& cannied, size_t maxsize)
        {
            RoadLine road(centerX, GetFirstPoint(cannied));
            optional<POINT> newPoint;
            totalGap = 0;
            while ((newPoint = GetNextPoint(cannied, road.getEstimatedAngle(), road.points.back(),pointDist)).valid && road.points.size() < maxsize)
            {
                if (!road.addPoint(*newPoint))
                    break;
            }
            road.totalGap = totalGap;

            return road;
        }
    };


}
