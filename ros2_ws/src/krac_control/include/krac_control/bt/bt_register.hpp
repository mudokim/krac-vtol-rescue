#pragma once

#include "krac_control/bt/mission_context.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>

namespace krac_control::bt
{
void registerKracBtNodes(BT::BehaviorTreeFactory& factory, const std::shared_ptr<MissionContext>& ctx);
}
