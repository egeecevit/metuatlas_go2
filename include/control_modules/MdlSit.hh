#ifndef SITMODULE_HH
#define SITMODULE_HH
#include <rtcore/Module.hh>
#include <vector>

#include "Eigen/Dense"

class MdlLegControl;
class QuadrupedKinematics;

namespace rtcore {
class Profiler;
}

#define SITMODULE_NAME "MdlSit"

class MdlSit : public rtcore::Module {
public:
  MdlSit();
  ~MdlSit();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();


private:
  bool _wait_done(double t);
  bool _sit_done(double t);
  bool _transition_done(double t);

  void _wait_entry();
  void _wait_during();
  void _wait_exit();
  

  void _sendTarget();
  void _setTargetInit();
  void _sendTargetAngle();
  void _setTargetAngle();
  void _computeProfile();
  void _getCurrentAngles();

  void _sit_entry();
  void _sit_during();
  void _sit_exit();

  void _transition_entry();
  void _transition_during();
  void _transition_exit();

  enum class _state_t { WAIT, TRANSITION, SIT, DONE };
  _state_t _state = _state_t::WAIT;

  double _mark = 0.0;

  Eigen::Vector3d _footpos[4];
  Eigen::Vector3d _footvel[4];
  Eigen::Vector3d _current_angles[4];

  MdlLegControl *_legs[4];
  QuadrupedKinematics *_kinematics = nullptr;

  double _origin[3] = {-0.05, 0.12, -0.26};

  rtcore::Profiler *_profiler[3][4];

  Eigen::Vector3d _footsitangle[4];
  Eigen::Vector3d _footsitangledot[4];

  std::vector<Eigen::Vector3d> _fangle;
};
#endif