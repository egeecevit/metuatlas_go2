#include "control_modules/MdlSit.hh"
#include "rtcore/ModuleManager.hh"
#include "rtcore/Profiler.hh"
#include <quadruped/MdlLegControl.hh>
#include <quadruped/QuadrupedKinematics.hh>

using namespace rtcore;

extern QuadrupedKinematics::params_t createGo2Config();

// Comment in/out beyond printf to enable/disable debug messages
#define DBGPRINT(...) printf(__VA_ARGS__)

MdlSit::MdlSit() : Module(SITMODULE_NAME, 0, SINGLE_USER) {
  DBGPRINT("MdlSit::MdlSit\n");
  for (int i = 0; i < 4; ++i) {
    _footsitangle[i] = Eigen::Vector3d::Zero();
    _footsitangledot[i] = Eigen::Vector3d::Zero();
  }
};

MdlSit::~MdlSit() {
  DBGPRINT("MdlSit::~MdlSit\n");
};

void MdlSit::init() {
  DBGPRINT("MdlSit::init\n");
  for (int l = 0; l < 4; l++)
    _legs[l] = (MdlLegControl *)_mgr->findModule(LEGMODULE_NAME, l);

  _kinematics = new QuadrupedKinematics(createGo2Config());

  for (int i = 0; i < 3; i++){
    _profiler[i] = new Profiler();
  }
}

void MdlSit::uninit() {
  DBGPRINT("MdlSit::uninit\n");
}

void MdlSit::activate() {
  DBGPRINT("MdlSit[%d]::activate\n", getIndex());
  for (int i = 0; i < 4; i++)
    _mgr->grabModule(_legs[i], this);

  _state = _state_t::WAIT;
  _mark = _mgr->readTime();
  _wait_entry();
}

void MdlSit::deactivate() {
  DBGPRINT("MdlSit::deactivate\n");

  for (int i = 0; i < 4; i++)
    _mgr->releaseModule(_legs[i], this);
}

bool MdlSit::_wait_done(double t) {
  return (t - _mark > 3);
}

bool MdlSit::_sit_done(double t) {
  return (t - _mark > 7);
}

bool MdlSit::_transition_done(double t) {
  return (t - _mark > 4);
}

void MdlSit::_wait_entry() {
  _setTargetInit();
}

void MdlSit::_wait_during() {
  _sendTarget();
}

void MdlSit::_wait_exit() {}

// implement transition methods here
void MdlSit::_transition_entry() {}

void MdlSit::_transition_during() {}

void MdlSit::_transition_exit() {}

void MdlSit::_sit_entry() {
  _setTargetAngle();
}

void MdlSit::_sit_during() {
  _sendTargetAngle();
}

void MdlSit::_sit_exit() {}


void MdlSit::_setTargetAngle() {
  for (int i = 0; i < 4; i++) {
    // Sitting joint angles (in radians)
    _footsitangle[i][0] = 0.5 * (i % 2 == 0 ? 1 : -1);  // Hip abduction: spread legs for stability
    _footsitangle[i][1] = 1.2;   // Hip flexion: bend hip forward significantly
    _footsitangle[i][2] = -2.7;  // Knee: bend knee to fold leg under body
    
    _footsitangledot[i] = Eigen::Vector3d::Zero();  // No velocity
  }
}

void MdlSit::_sendTargetAngle() {
  // Use angle control instead of position control
  for (int i = 0; i < 4; i++)
    _legs[i]->setTargetAngles(_footsitangle[i], _footsitangledot[i]);
}

void MdlSit::_sendTarget() {
  for (int i = 0; i < 4; i ++)
    _legs[i]->setTargetPosition(_footpos[i], _footvel[i]);
}

void MdlSit::_setTargetInit() {
  for (int i = 0; i < 4; i ++) {
    _footpos[i] = _kinematics->getKinematicParams().hip_positions.row(i).transpose();
    _footpos[i][0] += _origin[0];
    _footpos[i][1] += _origin[1] * (i % 2 == 0 ? 1 : -1);
    _footpos[i][2] += _origin[2];
    // This for loop initializes leg positions according to the kinematics so that it will stand upright initially
  }
}

void MdlSit::update() {
  double t = _mgr->readTime();

  for (int i = 0; i < 4; i++)
    _footvel[i] = Eigen::Vector3d::Zero();

  switch (_state)
  {
  case _state_t::WAIT:
    if (_wait_done(t)) {
      _state = _state_t::TRANSITION;
      _wait_exit();
      _transition_entry();
      break;
    }
    _wait_during();
    break;

  case _state_t::TRANSITION:
    if (_transition_done(t)) {
      _state = _state_t::SIT;
      _transition_exit();
      _sit_entry();
      break;
    }
    _transition_during();
    break;

  case _state_t::SIT:
    _sit_during();
    break;
    
  case _state_t::DONE:
    break;
  }
}