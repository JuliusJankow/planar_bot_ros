#ifndef __SSV_OBJECTS__
#define __SSV_OBJECTS__

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

#define TOL 1e-6

namespace am_ssv_dist
{
  class PSS_object;
  class LSS_object;
  class PSS_ref_object;

  class SSV_object
  {

    friend class SSV_DistCalculator;

  public:
    // don't change numbers!!!
    /// defines the possible ssv classes
    enum types{SSV=-1, PSS=0, LSS=1, TSS=2};
    /// defines the mirror planes (number indicates index in scale-vector)
    enum mirror_planes{YZ=0, XZ=1, XY=2, UNKNOWN=-1};

    // "normal" constructor
    SSV_object(types t, const int32_t id = -1) : typeID(t), id_ssv(id), radius_(0.0) {
      pt0 << 0,0,0,1;
    }
    SSV_object(types t, float r, const Vector4f &p0, const int32_t id = 0) : 
      typeID(t), id_ssv(id), radius_(r), pt0(p0) {}
      
    virtual ~SSV_object() {}
    
    // get and set methods
    float radius() const {return radius_;}
    void set_radius(const float r){radius_=r;}
    const Vector4f &p0() const { return pt0; }//wird in ssv bib nicht verwendet?
    void set_p0(const Vector4f &p) {pt0 = p;}
    types get_type() const {return typeID;}

    //! save Type ID for faster type recognition
    types typeID;

    //! id of each ssv object - identification for adding, modifying, ... (default value is -1)
    int32_t id_ssv;

  protected:
    float radius_;
    //! first point
    Vector4f pt0;
  };

  // caches informations needet by TSS_object and LSS_object (for internal use of tss only!)
  class PSS_ref_object
  {
  public:
    PSS_ref_object(const float &r, const Vector4f &pt0_in) :
      radius_(r),
      pt0(pt0_in)
    {}
    ~PSS_ref_object(){}

    const float &radius_;
    const Vector4f &pt0;
  };

  class PSS_object : public SSV_object
  {
    friend class SSV_DistCalculator;

  public:
    PSS_object() : SSV_object(PSS) {}
    ~PSS_object() {}
  };


  class LSS_object : public SSV_object
  {

    friend class SSV_DistCalculator;

  public:
    LSS_object() : SSV_object(LSS,0.1,Vector4f(0,0,0,1)),
                   pt1(0,0,0,1), e(0,0,0,1), ee(0),
                   pss0(radius_,pt0), pss1(radius_,pt1) {}
    ~LSS_object() {}
                  
    // getter and setters
    const Vector4f &p1() const { return pt1; }
    void set_p1(const Vector4f &p) {pt1 = p; recalc_e();}

  private:
    Vector4f pt1;

    Vector4f e;
    float ee;

    PSS_ref_object pss0;
    PSS_ref_object pss1;

    void recalc_e() {e=pt1-pt0; ee=e.squaredNorm();}
  };
} // namespace

#endif // __SSV_OBJECTS__
