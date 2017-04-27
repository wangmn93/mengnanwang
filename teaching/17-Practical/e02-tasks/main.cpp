#include <Roopi/roopi.h>
#include <Motion/komo.h>
#include <Control/taskControl.h>
#include <Gui/viewer.h>
#include <Kin/kinViewer.h>


//===============================================================================

void grasp(Roopi* r) {
}

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  Roopi R(false);
  R.setKinematics("../../../data/baxter_model/baxter.ors");

  R.startTweets();
  R.startTaskController();

  {
    auto home = R.home();
    R.wait(+home);
  }
  {
    // Move right shoulder .5 rads
    auto rArm = R.newCtrlTask(new TaskMap_qItself(QIP_byJointNames, {"right_s0"}, R.getK()), {}, {-0.5});
    R.wait(+rArm);
  }
  // Move left arm above box, and align
  {
    mlr::Body* obj1_body = R.getK()->getBodyByName("obj1");
    cout << "Body position: " <<  obj1_body->X << std::endl;

    auto lArm = R.newCtrlTask(new TaskMap_Default(posDiffTMT, R.getK(), "endeffL", NoVector, "obj1", NoVector), {}, {0., 0., .25}, {1e1});
    auto lAlign = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, R.getK(), "endeffL", Vector_y, "obj1", -Vector_z), {}, {}, {1e1});
    auto lAlign2 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, R.getK(), "endeffL", Vector_x, "obj1", Vector_x), {}, {}, {1e1});
    R.wait(lArm+lAlign);
    mlr::wait();
  }
  return 0;
}

