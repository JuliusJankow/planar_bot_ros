/*!
  @file ssv_dist_calc.hpp

  Description:

  Calculates the distance between two SSV objects. The result is saved
  in the \c SSV_DistCalcResult structure. Conventions:

  1) The \c start_point and \c end_point lays on the skeleton of the SSV
  objects and not on the outer hull.

  2) The squared distance calculation functions deliver only the
  squared distance between the skeletons (the radius must be
  substracted after the sqrt(squared distance), else if the SSV's
  penetrate each other the distance is negative and sqrt(dist<0)
  delivers NaN!)

  Copyright (C) 2010 Institute of Applied Mechanics, Technische Universitaet Muenchen.
  Author: Markus Schwienbacher (schwienbacher@amm.mw.tum.de)
*/

#ifndef __DIST_CALCULATOR__HPP__
#define __DIST_CALCULATOR__HPP__

#include <Eigen/Dense>
#include "ssv_objects.h"

using namespace Eigen;

namespace am_ssv_dist
{
  class SSV_DistCalculator;

  struct SSV_DistCalcResult
  {
    SSV_DistCalcResult() :
      distance(HUGE_VAL),
      start_point(0,0,0,1),
      end_point(0,0,0,1),
      con_vec(0,0,0,1)
    {}

    //! the resulting distance (= \c length(conVec) - radius_this - radius_other) (may be squared, depending on the function called)
    float distance;
    //! resulting start-point (point on body "from")
    Vector4f start_point;
    //! resulting end-point (point on body "to")
    Vector4f end_point;
    //! cached connecting vector from \c startPoint to \c endPoint (=\c endPoint - \c startPoint)
    Vector4f con_vec;
  };

  class SSV_DistCalculator
  {
  public:
    SSV_DistCalculator(){}
    ~SSV_DistCalculator(){}

    //! calculate distance between Point Swept Sphere (pss) and pss
    void cdPP(const SSV_object *p_from, const SSV_object *p_to, SSV_DistCalcResult &dr) const;
    //! calculate squared distance between pss and pss
    void cdsPP(const SSV_object *p_from, const SSV_object *p_to, SSV_DistCalcResult &drs) const;
    
    //! calculate distance between pss and Line Swept Sphere (lss)
    void cdPL(const SSV_object *p, const SSV_object *l, SSV_DistCalcResult &dr) const;
    //! calculate distance between lss and pss
    void cdLP(const SSV_object *l, const SSV_object *p, SSV_DistCalcResult &dr) const;
    //! calculate the squared distance between pss and lss
    void cdsPL(const SSV_object *p, const SSV_object *l, SSV_DistCalcResult &drs) const;
    //! calculate the squared distance between lss and pss
    void cdsLP(const SSV_object *l, const SSV_object *p, SSV_DistCalcResult &drs) const;
    
    //! calculate distance between line and line
    void cdLL(const SSV_object *l_from, const SSV_object *l_to, SSV_DistCalcResult &drs) const;
    //! calculate squared distance between lss and lss
    void cdsLL(const SSV_object *l_from, const SSV_object *l_to, SSV_DistCalcResult &drs) const;

    //! encodes the calculation direction
    enum direction_modes{from_to=0, to_from=1};

  private:
    
    //! for standard and reference
    template  <class P, class L>
    void cdsPL(const P *p, const L *l, SSV_DistCalcResult &drs, SSV_DistCalculator::direction_modes mode) const;
    
    //! work with reference
    template  <class LSS_T0, class LSS_T1>
    int cdsLL_ref(const LSS_T0 *lss0, const LSS_T1 *lss1, SSV_DistCalcResult &drs) const;
  }; // class

  inline void SSV_DistCalculator::cdPP(const SSV_object *p_from, const SSV_object *p_to, SSV_DistCalcResult &dr) const
  {
    cdsPP(p_from,p_to,dr);
    dr.distance=sqrt(dr.distance) - p_from->radius() - p_to->radius();
  }

  inline void SSV_DistCalculator::cdsPP(const SSV_object *p_from, const SSV_object *p_to, SSV_DistCalcResult &drs) const
  {
    drs.start_point = p_from->pt0;
    drs.end_point = p_to->pt0;
    drs.con_vec = p_to->pt0 - p_from->pt0;
    drs.distance = drs.con_vec.squaredNorm();
  }
  
  inline void SSV_DistCalculator::cdPL(const SSV_object *p, const SSV_object *l, SSV_DistCalcResult &dr) const
  {
    SSV_DistCalculator::cdsPL(p,l,dr,from_to);
    dr.distance=sqrt(dr.distance); // - p->radius() - l->radius();
  }

  inline void SSV_DistCalculator::cdLP(const SSV_object *l, const SSV_object *p, SSV_DistCalcResult &dr) const
  {
    SSV_DistCalculator::cdsPL(p,l,dr,to_from);
    dr.distance=sqrt(dr.distance); // - p->radius() - l->radius();
  }

  inline void SSV_DistCalculator::cdsPL(const SSV_object *p, const SSV_object *l,
                                        SSV_DistCalcResult &drs) const
  {
    cdsPL(p,l,drs,from_to);
  }

  inline void SSV_DistCalculator::cdsLP(const SSV_object *l, const SSV_object *p,
                                        SSV_DistCalcResult &drs) const
  {
    cdsPL(p,l,drs,to_from);
  }

  template <class P, class L>
  inline void SSV_DistCalculator::cdsPL(const P *pss_, const L *lss_,
                                        SSV_DistCalcResult &drs,
                                        SSV_DistCalculator::direction_modes mode) const
  {
    const PSS_object* pss = reinterpret_cast<const PSS_object*>(pss_);
    const LSS_object* lss = reinterpret_cast<const LSS_object*>(lss_);
  
    const Vector4f &l0=lss->pt0, &l1=lss->pt1, &e=lss->e, &p0=pss->pt0;
    const Vector4f c=p0-l0;
    const float ce=c.dot(e), &ee=lss->ee;

    if(mode == from_to)
      {
        drs.start_point=p0;
        // cases
        if(ce<0) { // left
          drs.end_point=l0;
        } else if(ce>ee) { // right
          drs.end_point=l1;
        } else { // middle
          const float f=ce/ee;
          drs.end_point=l0+f*e;
        }
      }
    else
      {
        drs.end_point=p0;
        // cases
        if(ce<0) { // left
          drs.start_point=l0;
        } else if(ce>ee) { // right
          drs.start_point=l1;
        } else { // middle
          const float f=ce/ee;
          drs.start_point=l0+f*e;
        }
      }
    drs.con_vec=drs.end_point-drs.start_point;
    drs.distance=drs.con_vec.squaredNorm();
  }

  inline void SSV_DistCalculator::cdLL(const SSV_object *l_from, const SSV_object *l_to, SSV_DistCalcResult &dr) const
  {
    cdsLL(l_from,l_to,dr);
    dr.distance=sqrt(dr.distance) - l_from->radius() - l_to->radius();
  }

  inline void SSV_DistCalculator::cdsLL(const SSV_object *l_from, const SSV_object *l_to, SSV_DistCalcResult &drs) const
  {
    const LSS_object *l0=(const LSS_object *)l_from;
    const LSS_object *l1=(const LSS_object *)l_to;
    cdsLL_ref(l0,l1,drs);
  }
  
  template  <class LSS_T0, class LSS_T1 >
  inline int SSV_DistCalculator::cdsLL_ref(const LSS_T0 *lss0, const LSS_T1 *lss1, SSV_DistCalcResult &drs) const
  {
    int parallel=0;
    const Vector4f &l00=lss0->pt0, &l10=lss1->pt0;
    const Vector4f &u=lss0->e, &v=lss1->e;
    const Vector4f w=l10-l00;

    const float uu = lss0->ee;
    const float vv = lss1->ee;
    const float uv = u.dot(v);
    const float uw = u.dot(w);
    const float vw = v.dot(w);
    // abbreviation for denominator in s and t, always > 0
    const float N = uu*vv - uv*uv;

    float s,t;

    // parallel
    if(N < TOL) {
      parallel=1;
      const float temp = uv+uw;
      const float ruv=1/(uv+uv);
      if(uw<0) { // first point left
        if(uv<-uw) {
          s = 0;
          t = uv<0?0:1;
        }
        else if(temp>uu) {
          s = 0.5;
          t = (uu-uw-uw)*ruv;
        }
        else {
          s = (vv+vw)*ruv;
          t = (uv-uw)*ruv;
        }
      }
      else if(uw>uu) { // first point right
        if(temp>uu) {
          s = 1;
          t = uv>0?0:1;
        }
        else if(temp<0) {
          s = 0.5;
          t = (uu-uw-uw)*ruv;
        }
        else {
          s = (uv+vv+vw)*ruv;
          t = (uv+uu-uw)*ruv;
        }
      }
      else { // first point over the middle
        if(temp>uu) {
          s = (uv+vw)*ruv;
          t = (uu-uw)*ruv;
        }
        else if(temp<0) {
          s = vw*ruv;
          t = -uw*ruv;
        }
        else {
          s = (vv+vw+vw)*ruv;
          t = 0.5;
        }
      }
    }
    // if skew, calculate distance for infinite lines
    else {
      parallel=0;
      float sZ = vv*uw - uv*vw;
      float tZ = uv*uw - uu*vw;
      float sN=N, tN=N;

      // first step
      if(tZ < 0.0) {
        sZ = uw;
        tZ = 0.0;
        sN = uu;
      }
      else if(tZ > tN) {
        sZ = uv+uw;
        tZ = tN;
        sN = uu;
      }

      // second step
      if(sZ < 0.0) {
        sZ = 0.0;
        if(vw > 0)
          tZ = 0;
        else if(-vw > vv)
          tZ = tN;
        else {
          tZ = -vw;
          tN = vv;
        }
      }
      else if(sZ > sN) {
        sZ = sN;
        if(uv-vw < 0)
          tZ = 0;
        else if(uv-vw > vv)
          tZ = tN;
        else {
          tZ = uv-vw;
          tN = vv;
        }
      }

      // calculation of actual distance by using both intersection points
      s = sZ/sN;
      t = tZ/tN;
    }

    drs.start_point = lss0->pt0 + s*u;
    drs.end_point = lss1->pt0 + t*v;
    drs.con_vec=drs.end_point-drs.start_point;
    drs.distance = drs.con_vec.squaredNorm();
    
    return parallel;
  }

} // namespace am_ssv_dist

#endif // __DIST_CALCULATOR__HPP__
