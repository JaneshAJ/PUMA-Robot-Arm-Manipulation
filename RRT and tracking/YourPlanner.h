#ifndef _YOUR_PLANNER_H_
#define _YOUR_PLANNER_H_

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

#include "RrtConConBase.h"

using namespace ::rl::plan;

/**
*	The implementation of your planner.
*	modify any of the existing methods to improve planning performance.
*/
class YourPlanner : public RrtConConBase
{
public:
  YourPlanner();

  virtual ~YourPlanner();

  virtual ::std::string getName() const;

  bool solve();

protected:
  void choose(::rl::math::Vector& chosen);

  ::rl::math::Real calc_distance(const ::rl::math::Vector q1,const ::rl::math::Vector q2);

  void find_k_nearest(Tree& tree, const ::rl::math::Vector chosen, std::pair<RrtConConBase::Vertex, ::rl::math::Real> *k_nearest, int k);

  RrtConConBase::Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

  RrtConConBase::Vertex extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

private:
    bool swap_trees;

    bool grow_from_start;

    ::rl::math::Vector end_q;

};

#endif // _YOUR_PLANNER_H_
