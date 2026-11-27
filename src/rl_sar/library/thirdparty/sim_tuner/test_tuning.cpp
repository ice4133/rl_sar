// SPDX-License-Identifier: Apache-2.0
#include "tuning.hpp"
#include "transport.hpp"
#include "mujoco_joint_drive.hpp"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <iostream>
#include <thread>

namespace {
void Require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
bool Near(double a, double b) { return std::abs(a-b) < 1e-9; }
const char* kXml = R"(<mujoco>
  <option timestep="0.0025" integrator="implicitfast" gravity="0 0 0"/>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 .1" friction=".6 .005 .0001"/>
    <body name="base" pos="0 0 1">
      <joint name="wheel" axis="0 0 1" damping=".1" armature=".01"/>
      <inertial pos="0 0 0" mass="1" diaginertia=".1 .1 .1"/>
      <geom name="wheel_geom" type="sphere" size=".1" friction=".8 .005 .0001"/>
    </body>
  </worldbody>
  <contact><pair geom1="floor" geom2="wheel_geom" friction=".9 .7 .005 .0001 .0001"/></contact>
  <actuator><motor joint="wheel" gear="1" ctrllimited="true" ctrlrange="-2 2"/></actuator>
  <sensor><jointpos joint="wheel"/><jointvel joint="wheel"/><jointactuatorfrc joint="wheel"/></sensor>
</mujoco>)";
sim_tuner::Command Command(double force) { return {{0},{0},{0},{0},{force},{2}}; }
}
int main(int argc, char** argv) {
    try {
        mjVFS vfs; mj_defaultVFS(&vfs);
        mj_addBufferVFS(&vfs,"tuner.xml",kXml,std::char_traits<char>::length(kXml));
        char error[1024]{};
        std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model(mj_loadXML("tuner.xml",&vfs,error,sizeof(error)),mj_deleteModel);
        mj_deleteVFS(&vfs); Require(bool(model),error);
        std::unique_ptr<mjData,decltype(&mj_deleteData)> data(mj_makeData(model.get()),mj_deleteData);
        mjModel* m = model.get(); mjData* d = data.get(); mj_forward(m,d);
        sim_tuner::Tuning tuning(m,d,{0},{"wheel"},"test","implicit","base");
        int resets = 0;
        auto reset = [&]() { ++resets; mj_resetData(m,d); mj_forward(m,d); tuning.Reset(); };
        long long id = 0;
        auto request = [&](const std::string& patch, bool do_reset = false) {
            return YAML::Load(tuning.Handle("{\"protocol\":1,\"id\":"+std::to_string(++id)+
                ",\"op\":\"set\",\"reset\":"+(do_reset ? "true" : "false")+",\"params\":"+patch+"}",true,reset));
        };
        if (argc == 3 && std::string(argv[1]) == "--serve") {
            sim_tuner::Transport transport(std::stoi(argv[2]));
            std::cout << "READY\n" << std::flush;
            for (int i = 0; i < 60000; ++i) {
                for (auto& message : transport.TakeRequests())
                    transport.Reply({message.connection,tuning.Handle(message.text,true,reset)});
                tuning.Submit(Command(std::sin(d->time))); tuning.Tick(); mj_step(m,d);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            return 0;
        }

        // Whole transactions are rejected, including valid fields preceding an invalid one.
        Require(!request("{\"kp_scale\":2,\"unknown\":1}")["ok"].as<bool>(),"Unknown key accepted");
        Require(Near(tuning.values().at("kp_scale"),1),"Partial transaction applied");
        Require(!request("{\"delay_ms\":-1}")["ok"].as<bool>(),"Negative delay accepted");
        Require(!request("{\"noise_seed\":1.5}")["ok"].as<bool>(),"Fractional seed accepted");
        Require(!request("{\"gyro_noise\":.nan}")["ok"].as<bool>(),"NaN accepted");
        Require(!YAML::Load(tuning.Handle("garbage",true,reset))["ok"].as<bool>(),"Malformed request accepted");

        Require(request("{\"friction_override\":1,\"sliding_friction\":.2}")["ok"].as<bool>(),"Friction set failed");
        Require(Near(m->geom_friction[0],.2) && Near(m->geom_friction[3],.2),"Wheel/floor mismatch");
        Require(Near(m->pair_friction[0],.2) && Near(m->pair_friction[1],.2),"Explicit pair not updated");
        request("{\"friction_override\":0}");
        Require(Near(m->geom_friction[0],.6) && Near(m->geom_friction[3],.8),"Original friction not restored");
        Require(Near(m->pair_friction[0],.9) && Near(m->pair_friction[1],.7),"Original pair not restored");
        request("{\"contact_time_scale\":2,\"joint_damping_add\":.2,\"joint_friction_add\":.1}");
        Require(Near(m->geom_solref[0],.04) && Near(m->pair_solref[0],.04),"Contact time not scaled");
        Require(Near(m->dof_damping[0],.3) && Near(m->dof_frictionloss[0],.1),"Passive joint settings failed");

        Require(!request("{\"mass_scale\":2}")["ok"].as<bool>() && Near(m->body_mass[1],1),"Mass changed without reset");
        Require(request("{\"mass_scale\":2,\"inertia_scale\":1.5,\"com_x\":.02,\"armature_add\":.01}",true)["ok"].as<bool>(),"Mass transaction failed");
        Require(resets == 1 && Near(m->body_mass[1],2) && Near(m->body_inertia[3],.3),"Mass/inertia wrong");
        Require(Near(m->body_subtreemass[1],2) && Near(m->body_ipos[3],.02) && Near(m->dof_armature[0],.02),"Derived constants/COM wrong");
        request("{\"delay_ms\":10}");
        tuning.Submit(Command(1)); tuning.Tick(); Require(Near(d->ctrl[0],0),"Delay applied too early");
        for (int i = 0; i < 50; ++i) tuning.Submit(Command(1));
        Require(tuning.queued() == 1,"Paused submissions grew the delay queue");
        d->time = .0075; tuning.Tick(); Require(Near(d->ctrl[0],0),"Delay released before deadline");
        d->time = .01; tuning.Tick(); Require(Near(d->ctrl[0],1),"Delayed packet not released");
        tuning.Submit(Command(2)); tuning.Tick(); Require(Near(d->ctrl[0],1),"New packet bypassed delay");
        d->time = .02; tuning.Tick(); Require(Near(d->ctrl[0],2),"Delayed FIFO wrong");
        request("{\"motor_strength\":.5,\"torque_limit_scale\":.5}"); tuning.Tick();
        Require(Near(d->ctrl[0],1) && Near(m->actuator_forcerange[1],1),"Strength/limit wrong");
        tuning.Submit(Command(1)); request("{\"delay_ms\":20}");
        Require(tuning.queued() == 0,"Changing delay did not discard in-flight packets");
        tuning.Tick(); Require(Near(d->ctrl[0],1),"Changing delay did not hold last applied command");
        d->time = 0; tuning.Tick(); Require(tuning.queued() == 0 && Near(d->ctrl[0],0),"Rewind leaked an old command");

        request("{\"joint_pos_noise\":.01,\"gyro_noise\":.02,\"gyro_bias_z\":.1}");
        std::vector<float> q{0},dq{0},gyro(3,0); tuning.Reset(); tuning.Perturb(q,dq,gyro);
        auto first_q = q; auto first_gyro = gyro;
        q = {0}; dq = {0}; gyro.assign(3,0); tuning.Reset(); tuning.Perturb(q,dq,gyro);
        Require(q == first_q && gyro == first_gyro && Near(d->qpos[0],0),"Noise not reproducible or modified physics");
        auto restored = YAML::Load(tuning.Handle("{\"protocol\":1,\"id\":99,\"op\":\"defaults\",\"reset\":true}",false,reset));
        Require(restored["ok"].as<bool>() && Near(m->body_mass[1],1) && Near(m->geom_friction[3],.8),"Defaults failed");
        Require(Near(m->dof_damping[0],.1) && Near(m->dof_armature[0],.01) && Near(m->geom_solref[0],.02),"Baseline did not restore");
        Require(restored["params"]["delay_ms"].as<double>() == 0 && restored["joint_names"][0].as<std::string>() == "wheel","Snapshot wrong");
        std::cout << "PASS: transactional validation, geom/pair friction, model constants/reset, simulation-time FIFO, paused queue, strength/limits, rewind, noise and baseline restore\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << "FAIL: " << error.what() << '\n'; return 1; }
}
