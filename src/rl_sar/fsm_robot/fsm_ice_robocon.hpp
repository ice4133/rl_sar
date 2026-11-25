/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ICE_ROBOCON_FSM_HPP
#define ICE_ROBOCON_FSM_HPP

#include "fsm_go2w.hpp"

namespace ice_robocon_fsm
{

// Robocon's CAD frames already contain the bent-leg geometry. Stand directly
// in its configured pose instead of using Go2W's hard-coded crouch angles.
class RLFSMStateGetUp : public go2w_fsm::RLFSMStateGetUp
{
public:
    using go2w_fsm::RLFSMStateGetUp::RLFSMStateGetUp;

    void Run() override
    {
        Interpolate(percent_getup, rl.now_state.motor_state.q,
                    rl.params.Get<std::vector<float>>("default_dof_pos"),
                    2.0f, "Getting up", true);
    }
};

} // namespace ice_robocon_fsm

class IceRoboconFSMFactory : public Go2WFSMFactory
{
public:
    using Go2WFSMFactory::Go2WFSMFactory;

    std::string GetType() const override { return "ice/robocon"; }

    std::shared_ptr<FSMState> CreateState(void* context, const std::string& state_name) override
    {
        if (state_name == "RLFSMStateGetUp")
        {
            return std::make_shared<ice_robocon_fsm::RLFSMStateGetUp>(static_cast<RL*>(context));
        }
        return Go2WFSMFactory::CreateState(context, state_name);
    }
};

REGISTER_FSM_FACTORY(IceRoboconFSMFactory, "RLFSMStatePassive")

#endif // ICE_ROBOCON_FSM_HPP
