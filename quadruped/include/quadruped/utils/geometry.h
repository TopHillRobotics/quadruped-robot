#ifndef ASCEND_MATH_GEOMETRY_H
#define ASCEND_MATH_GEOMETRY_H

#include <cmath>
#include <iostream>
#include <type_traits>
#include "utils/algebra.h"
#include "utils/cppTypes.h"
#include "utils/se3.h"

namespace robotics {
    namespace math {

        /** 
         * @brief Linear interpolate between two point in state space.
         * @example 
         *  robotics::math::Segment<float, Vec6<float>> segment;
         *  Vec6<float> s = Vec6<float>::Zero();
         *  Vec6<float> t;
         *  t<< 1, 1, 1, 1, 0, 0;
         *  segment.Reset(s,t);
         *  Vec6<float> midpoint = segment.GetPoint(0.9);
         *  cout << midpoint << endl;
         * 
         */
        template <class t=float, class T=Vec6<t>>
        class Segment {
            public:
                Segment() {}
                Segment(T source_, T dest_) {
                    source = source_;
                    dest = dest_;
                }
                ~Segment() = default;
                T GetPoint(float phase) {
                    if (phase > 1.0) {
                        phase = 1.0;
                    } else if (phase < 0.0) {
                        phase =0.0;
                    }
                    // std::cout << "phase " << phase << " source " << source.transpose() << " dest " << dest.transpose() <<std::endl;
                    T point = phase*dest + (1.0-phase)*source;
                    return point;
                }
                void Reset(T source_, T dest_) {
                    source = source_;
                    dest = dest_;
                }
            private:
                T source;
                T dest;
        };


        /** @brief 
         * @ref https://math.stackexchange.com/questions/2217654/interpolation-in-so3-different-approaches
         * @ref https://math.stackexchange.com/questions/1832019/what-is-the-angular-velocity-in-an-inertial-frame-given-the-angular-velocity-in
         * 
         */
        class CubicSplineInSO3 {
        public:
            CubicSplineInSO3() {}

            CubicSplineInSO3(Mat3<float> R1, Mat3<float>R2, float t0, float duration, Mat3<float> inertial);

            ~CubicSplineInSO3() = default;

            void GetPoint(float currentTime, Mat3<float>& outR, Vec3<float>& outwb);

        private:
            float t0;
            float dt;
            float lastTime;
            Mat3<float> R1;
            Mat3<float> lastR;
            Mat3<float> R2;
            Mat3<float> dR1;
            Mat3<float> dR2;

            Mat3<float> M0;
            Mat3<float> M1;
            Mat3<float> M2;
            Mat3<float> M3;
            Mat3<float> W;
        };

        /**
         * @brief Spline class defines an abstract class for different spline interpolations
         */
        class Spline {
        public:
            /**
             * @brief record the spatial information, including the time dervative.
             */
            struct Point {
                Point() : x(0.0), xd(0.0), xdd(0.0)
                {}

                Point(float p, float v = 0.0, float a = 0.0) : x(p), xd(v), xdd(a)
                {}

                void setZero()
                {
                    x = 0.0;
                    xd = 0.0;
                    xdd = 0.0;
                }
                friend std::ostream &operator <<(std::ostream& cout, Point& p);
        
                float x;
                float xd;
                float xdd;
            };

            Spline::Point operator=(const Spline::Point &rhs)
            {
                Spline::Point out;
                out.x = rhs.x;
                out.xd = rhs.xd;
                out.xdd = rhs.xdd;
                return out;
            }

            /** @brief Constructor function */
            Spline();

            /**
             * @brief Constructor function
             * @param const float& Initial time
             * @param const float& Duration of the spline
             * @param const Point& Start point
             * @param const Point& End point
             */
            Spline(const float &initial_time,
                   const float &duration,
                   const Point &start,
                   const Point &end);
            
            Spline(const float &initial_time,
                    const float &duration,
                    float start_p,
                    float end_p);

            /** @ Destructor function */
            virtual ~Spline() = default;

            /**
             * @brief Sets the boundary of the spline
             * @param const float& Initial time
             * @param const float& Duration of the spline
             * @param const Point& Start point
             * @param const Point& End point
             */
            void setBoundary(const float &initial_time,
                             const float &duration,
                             const Point &start_p,
                             const Point &end_p);

            /**
             * @brief Sets the boundary of the spline
             * @param const float& Initial time
             * @param const float& Duration of the spline
             * @param const float& Start point
             * @param const float& End point
             */
            void setBoundary(const float &initial_time,
                             const float &duration,
                             const float &start_p,
                             const float &end_p);

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param Point& Point value
             */
            virtual bool getPoint(const float &current_time,
                                  Point &p) = 0;

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param float& Point value
             */
            virtual bool getPoint(const float &current_time,
                                  float &p) = 0;

            bool isTimeElapsed(float &time);

        protected:
            /** @brief Initial time of the spline */
            float initial_time_;

            /** @brief Duration of the spline */
            float duration_;

            /** Start point of the spline */
            Point start_;

            /** @brief End point of the spline */
            Point end_;
        };

        inline bool Spline::isTimeElapsed(float &t)
        {
            //this makes sense only without the time interval
            if ((t - initial_time_) > duration_)
                return true;
            else
                return false;
        }

        
        class QuadraticSpline : public Spline {
            public:
            /** @brief Constructor function */
            QuadraticSpline();

            /**
             * @brief Constructor function
             * @param const float& Initial time
             * @param const float& Duration of the spline
             * @param const Point& Start point
             * @param const Point& End point
             */
            QuadraticSpline(const float &initial_time,
                        const float &duration,
                        const Point &start,
                        const Point &end);

            QuadraticSpline(const float &initial_time,
                        const float &duration,
                        float start,
                        float end);

            /** @ Destructor function */
            ~QuadraticSpline() = default;

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param Point& Point value
             */
            bool getPoint(const float &current_time, float mid,
                          Point &p);

            virtual bool getPoint(const float &current_time, Point &p) {}

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param float& Point value
             */
            bool getPoint(const float &current_time, float mid,
                          float &p);

            virtual bool getPoint(const float &current_time, float &p) {};
        };
        
        
        /**
         * @brief CubicSpline class defines a cubic spline interpolation
         */
        class CubicSpline : public Spline {
        public:
            /** @brief Constructor function */
            CubicSpline();

            /**
             * @brief Constructor function
             * @param const float& Initial time
             * @param const float& Duration of the spline
             * @param const Point& Start point
             * @param const Point& End point
             */
            CubicSpline(const float &initial_time,
                        const float &duration,
                        const Point &start,
                        const Point &end);

            CubicSpline(const float &initial_time,
                        const float &duration,
                        float start,
                        float end);

            /** @ Destructor function */
            ~CubicSpline();

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param Point& Point value
             */
            bool getPoint(const float &current_time,
                          Point &p);

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param float& Point value
             */
            bool getPoint(const float &current_time,
                          float &p);
        };

        /**
         * @brief FifthOrderPolySpline class defines a 5-order spline interpolation
         */
        class FifthOrderPolySpline : public Spline {
        public:
            /** @brief Constructor function */
            FifthOrderPolySpline();

            /**
             * @brief Constructor function
             * @param const float& Initial time
             * @param const float& Duration of the spline
             * @param const Point& Start point
             * @param const Point& End point
             */
            FifthOrderPolySpline(const float &initial_time,
                                 const float &duration,
                                 const Point &start,
                                 const Point &end);

            /** @ Destructor function */
            ~FifthOrderPolySpline();

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param Point& Point value
             */
            bool getPoint(const float &current_time,
                          Point &p);

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param float& Point value
             */
            bool getPoint(const float &current_time,
                          float &p);
        };

        /**
         * @brief LinearSpline class defines a linear spline interpolation
         */
        class LinearSpline : public Spline {
        public:
            /** @brief Constructor function */
            LinearSpline();

            /**
             * @brief Constructor function
             * @param const float& Initial time
             * @param const float& Duration of the spline
             * @param const Point& Start point
             * @param const Point& End point
             */
            LinearSpline(const float &initial_time,
                         const float &duration,
                         const Point &start,
                         const Point &end);

            /** @ Destructor function */
            ~LinearSpline();

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param Point& Point value
             */
            bool getPoint(const float &current_time, Point &p);

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param float& Point value
             */
            bool getPoint(const float &current_time, float &p);
        };
        
        /*!
        * Linear interpolation between y0 and yf.  x is between 0 and 1
        */
        template <typename y_t, typename x_t>
        y_t lerp(y_t y0, y_t yf, x_t x) {
            static_assert(std::is_floating_point<x_t>::value,
                            "must use floating point value");
            assert(x >= 0 && x <= 1);
            return y0 + (yf - y0) * x;
        };

        /*!
        * Cubic bezier interpolation between y0 and yf.  x is between 0 and 1
        */
        template <typename y_t, typename x_t>
        y_t cubicBezier(y_t y0, y_t yf, x_t x) {
            static_assert(std::is_floating_point<x_t>::value,
                            "must use floating point value");
            assert(x >= 0 && x <= 1);
            y_t yDiff = yf - y0;
            x_t bezier = x * x * x + x_t(3) * (x * x * (x_t(1) - x));
            return y0 + bezier * yDiff;
        };

        /*!
        * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and
        * 1
        */
        template <typename y_t, typename x_t>
        y_t cubicBezierFirstDerivative(y_t y0, y_t yf, x_t x) {
            static_assert(std::is_floating_point<x_t>::value,
                            "must use floating point value");
            assert(x >= 0 && x <= 1);
            y_t yDiff = yf - y0;
            x_t bezier = x_t(6) * x * (x_t(1) - x);
            return bezier * yDiff;
        };

        /*!
        * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and
        * 1
        */
       template <typename y_t, typename x_t>
        y_t cubicBezierSecondDerivative(y_t y0, y_t yf, x_t x) {
            static_assert(std::is_floating_point<x_t>::value,
                            "must use floating point value");
            assert(x >= 0 && x <= 1);
            y_t yDiff = yf - y0;
            x_t bezier = x_t(6) - x_t(12) * x;
            return bezier * yDiff;
        };
    } //  namespace math
} // namespace robotics

#endif//ASCEND_MATH_GEOMETRY_H
