#include <Roopi/roopi.h>
#include <Motion/komo.h>
#include <Control/taskControl.h>
#include <Gui/viewer.h>
#include <Kin/kinViewer.h>
#include <Geo/geo.h>


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
      //getchar();
    // Move right shoulder .5 rads
    auto lglf = R.newCtrlTask(new TaskMap_qItself(QIP_byJointNames, {"l_gripper_l_finger_joint",
                             "l_gripper_r_finger_joint"}, R.getK()), {}, {.5, -.0});
    //auto test = R.newCtrlTask();
    R.wait(+lglf);
    //test
    //auto rGripper = R.newCtrlTask(new TaskMap_qItself(QIP_byJointNames, {"right_w0"}, R.getK()), {}, {-0.5});
   // R.wait(+lglf);
      //auto gripperR =  R.newCtrlTask(new TaskMap_qItself(QIP_byJointNames, {"l_gripper_r_finger_joint"}, R.getK()), {}, {-.5});
      //auto gripper2R = R.newCtrlTask(new TaskMap_qItself(QIP_byJointNames, {"l_gripper_l_finger_joint"}, R.getK()), {}, {.5});

  }

  // Move left arm above box, and align
  {
    mlr::Body* obj1_body = R.getK()->getBodyByName("obj1");
    cout << "Body position: " <<  obj1_body->X << std::endl;

    //Script_setGripper(R, LR_left, .08);
    //Script_setGripper(R, LR_left, .0);

    mlr::Vector v1,v2;
    v1.set(.0,.0,.5);
    v1.normalize();
    v2.set(.5,.0,.0);
    v2.normalize();

    auto lAlign = R.newCtrlTask(new TaskMap_Default(vecDiffTMT, R.getK(), "endeffL", v2, NULL, v1), {}, {}, {1e1});
    //lAlign->task->PD().v_target={v1(0),v1(1),v2(2)};
    R.wait(+lAlign);
    double target[3] = {.0,.0,.75};
    double PI =3.14;
    for( int a = 0; a < 5; a = a + 1 ) {

          target[0] = ::cos(2*PI/10*a)*.2;
          target[1] = ::sin(2*PI/10*a)*.2;

          //cout<<"x,y"<<target[0]<<target[1]<<endl;
          //auto lArm = R.newCtrlTask(new TaskMap_Default(posDiffTMT, R.getK(), "endeffL", NoVector, "obj1", NoVector), {}, {target[0],target[1],target[2]}, {1e1});
            //lArm->task->PD().maxAcc=1;
            //lArm->start();
          //R.wait(+lArm);
       }


    //auto gripperR =  R.newCtrlTask(new TaskMap_qItself(QIP_byJointNames, {"right_w0"}, R.getK()), {}, {.5});
    //auto lAlign2 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, R.getK(), "endeffL", Vector_x, "obj1", Vector_x), {}, {}, {1e1});


    arr a = R.get_q0();
    for(double q :a) cout<<"joint"<<q<<endl;
    arr qs = R.getK()->q;
    cout<<"------------"<<endl;
    mlr::Joint* shoulderR = R.getK()->getJointByName("right_s0");
    cout<<qs(shoulderR->qIndex)<<endl;
    auto rs = R.newCtrlTask(new TaskMap_qItself(QIP_byJointNames, {"right_s0","right_s1"}, R.getK()), {}, {.5,.5});
    R.wait(+rs);
    //R.wait(+lArm);
    mlr::wait();
  }

  return 0;
}

