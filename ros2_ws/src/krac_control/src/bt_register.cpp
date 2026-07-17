#include "krac_control/bt/bt_register.hpp"
#include "krac_control/bt/bt_conditions.hpp"
#include "krac_control/bt/bt_actions_basic.hpp"
#include "krac_control/bt/bt_actions_mission.hpp"
#include "krac_control/bt/bt_actions_vision.hpp"
#include "krac_control/bt/bt_actions_recovery.hpp"
#include "krac_control/bt/bt_actions_precision.hpp"

namespace krac_control::bt
{

void registerKracBtNodes(BT::BehaviorTreeFactory& factory, const std::shared_ptr<MissionContext>& ctx)
{
  setGlobalContext(ctx);

  factory.registerNodeType<IsEmergencyDetected>("IsEmergencyDetected");
  factory.registerNodeType<EmergencyRecovery>("EmergencyRecovery");
  factory.registerNodeType<GlobalMissionRecovery>("GlobalMissionRecovery");
  factory.registerNodeType<RecoverVisionLanding>("RecoverVisionLanding");
  factory.registerNodeType<RecoverFinalLanding>("RecoverFinalLanding");
  factory.registerNodeType<FailAfterRecovery>("FailAfterRecovery");

  factory.registerNodeType<IsMAVROSConnected>("IsMAVROSConnected");
  factory.registerNodeType<IsGPSReady>("IsGPSReady");
  factory.registerNodeType<IsGlobalPositionFresh>("IsGlobalPositionFresh");
  factory.registerNodeType<IsBatterySafe>("IsBatterySafe");
  factory.registerNodeType<IsOffboardSetpointStreamAlive>("IsOffboardSetpointStreamAlive");
  factory.registerNodeType<IsFlightMode>("IsFlightMode");
  factory.registerNodeType<IsVehicleArmed>("IsVehicleArmed");
  factory.registerNodeType<IsVTOLStateMC>("IsVTOLStateMC");
  factory.registerNodeType<IsAltitudeReached>("IsAltitudeReached");
  factory.registerNodeType<IsMissionUploaded>("IsMissionUploaded");
  factory.registerNodeType<IsRescueCompleted>("IsRescueCompleted");
  factory.registerNodeType<IsAtREPRegion>("IsAtREPRegion");
  factory.registerNodeType<IsAtHomeRegion>("IsAtHomeRegion");
  factory.registerNodeType<IsVisionTopicFresh>("IsVisionTopicFresh");
  factory.registerNodeType<IsObjectDetected>("IsObjectDetected");

  factory.registerNodeType<StartOffboardSetpointStream>("StartOffboardSetpointStream");
  factory.registerNodeType<ExecuteExternalRescueModule>("ExecuteExternalRescueModule");
  factory.registerNodeType<SetFlightMode>("SetFlightMode");
  factory.registerNodeType<ArmVehicle>("ArmVehicle");
  factory.registerNodeType<CommandVTOLTransition>("CommandVTOLTransition");
  factory.registerNodeType<ClearMissionPlan>("ClearMissionPlan");
  factory.registerNodeType<UploadMissionPlan>("UploadMissionPlan");
  factory.registerNodeType<ActivateYOLO>("ActivateYOLO");
  factory.registerNodeType<DeactivateYOLO>("DeactivateYOLO");
  factory.registerNodeType<CloseGripper>("CloseGripper");

  factory.registerNodeType<FlyToAltitude>("FlyToAltitude");
  factory.registerNodeType<WaitForHoverStable>("WaitForHoverStable");
  factory.registerNodeType<EnsureFixedWingCruise>("EnsureFixedWingCruise");
  factory.registerNodeType<EnterREPStation>("EnterREPStation");
  factory.registerNodeType<EnterHomeStation>("EnterHomeStation");
  factory.registerNodeType<DetectLanding>("DetectLanding");
  factory.registerNodeType<VerifyLandingComplete>("VerifyLandingComplete");
  factory.registerNodeType<VerifyPayloadSecured>("VerifyPayloadSecured");

  factory.registerNodeType<ExecuteSearchPattern>("ExecuteSearchPattern");
  factory.registerNodeType<AlignToBasket>("AlignToBasket");
  factory.registerNodeType<AlignToLandingMarker>("AlignToLandingMarker");
  factory.registerNodeType<DescendWithAlignment>("DescendWithAlignment");
  factory.registerNodeType<LandOnBasketZone>("LandOnBasketZone");
  factory.registerNodeType<LandOnLandingZone>("LandOnLandingZone");
  factory.registerNodeType<VerifyBasketPicked>("VerifyBasketPicked");

  factory.registerNodeType<IsWaypointReached>("IsWaypointReached");
  factory.registerNodeType<WaitForWaypointReached>("WaitForWaypointReached");
  factory.registerNodeType<SetMissionCurrentWaypoint>("SetMissionCurrentWaypoint");
  factory.registerNodeType<EnablePrecisionLander>("EnablePrecisionLander");
  factory.registerNodeType<IsPrecisionTargetDetected>("IsPrecisionTargetDetected");
  factory.registerNodeType<WaitForPrecisionTarget>("WaitForPrecisionTarget");
  factory.registerNodeType<PrecisionLandOnTarget>("PrecisionLandOnTarget");
  factory.registerNodeType<FlyToLocalPoint>("FlyToLocalPoint");
  factory.registerNodeType<AlignHeadingToWaypoint>("AlignHeadingToWaypoint");
  factory.registerNodeType<SetMissionPhase>("SetMissionPhase");
  factory.registerNodeType<OpenGripper>("OpenGripper");
}

}  // namespace krac_control::bt
