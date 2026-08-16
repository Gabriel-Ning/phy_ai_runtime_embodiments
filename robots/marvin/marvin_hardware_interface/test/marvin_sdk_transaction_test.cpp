// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <vector>

#include "marvin_hardware_interface/marvin_sdk_bridge.hpp"

namespace
{

struct FakeSdkState
{
  bool clear_ok{true};
  bool target_a_ok{true};
  bool target_b_ok{true};
  bool command_a_ok{true};
  bool command_b_ok{true};
  bool send_ok{true};
  int send_failures_remaining{0};
  int clear_calls{0};
  int target_a_calls{0};
  int target_b_calls{0};
  int command_a_calls{0};
  int command_b_calls{0};
  int send_calls{0};
};

FakeSdkState fake;

void reset_fake()
{
  fake = FakeSdkState{};
}

}  // namespace

namespace marvin_sdk_bridge
{

extern "C"
{

bool OnClearSet()
{
  fake.clear_calls++;
  return fake.clear_ok;
}

bool OnSetSend()
{
  fake.send_calls++;
  if (fake.send_failures_remaining > 0)
  {
    fake.send_failures_remaining--;
    return false;
  }
  return fake.send_ok;
}

bool OnSetTargetState_A(int)
{
  fake.target_a_calls++;
  return fake.target_a_ok;
}

bool OnSetTargetState_B(int)
{
  fake.target_b_calls++;
  return fake.target_b_ok;
}

bool OnSetJointCmdPos_A(double[7])
{
  fake.command_a_calls++;
  return fake.command_a_ok;
}

bool OnSetJointCmdPos_B(double[7])
{
  fake.command_b_calls++;
  return fake.command_b_ok;
}

bool OnGetBuf(DCSS * frame)
{
  frame->m_State[0].m_CurState = ARM_STATE_IDLE;
  frame->m_State[1].m_CurState = ARM_STATE_IDLE;
  return true;
}

}  // extern "C"

}  // namespace marvin_sdk_bridge

TEST(SdkTransaction, BimanualExitUsesOneDispatch)
{
  reset_fake();

  EXPECT_TRUE(marvin_sdk_bridge::exit_position_modes(true, true));
  EXPECT_EQ(fake.clear_calls, 1);
  EXPECT_EQ(fake.target_a_calls, 1);
  EXPECT_EQ(fake.target_b_calls, 1);
  EXPECT_EQ(fake.send_calls, 1);
}

TEST(SdkTransaction, BimanualExitDoesNotSendPartialTransaction)
{
  reset_fake();
  fake.target_b_ok = false;

  EXPECT_FALSE(marvin_sdk_bridge::exit_position_modes(true, true));
  EXPECT_EQ(fake.target_a_calls, 1);
  EXPECT_EQ(fake.target_b_calls, 1);
  EXPECT_EQ(fake.send_calls, 0);
}

TEST(SdkTransaction, BimanualExitRetriesBusySendAsCompleteTransaction)
{
  reset_fake();
  fake.send_failures_remaining = 1;

  EXPECT_TRUE(marvin_sdk_bridge::exit_position_modes(true, true));
  EXPECT_EQ(fake.clear_calls, 2);
  EXPECT_EQ(fake.target_a_calls, 2);
  EXPECT_EQ(fake.target_b_calls, 2);
  EXPECT_EQ(fake.send_calls, 2);
}

TEST(SdkTransaction, PositionDispatchStagingError)
{
  reset_fake();
  fake.command_b_ok = false;
  double left[7]{};
  double right[7]{};

  EXPECT_EQ(marvin_sdk_bridge::dispatch_position_commands(left, true, right, true),
            marvin_sdk_bridge::DispatchResult::StagingError);
  EXPECT_EQ(fake.command_a_calls, 1);
  EXPECT_EQ(fake.command_b_calls, 1);
  EXPECT_EQ(fake.send_calls, 0);
}

TEST(SdkTransaction, PositionDispatchSendBusy)
{
  reset_fake();
  fake.send_ok = false;
  double left[7]{};
  double right[7]{};

  EXPECT_EQ(marvin_sdk_bridge::dispatch_position_commands(left, true, right, true),
            marvin_sdk_bridge::DispatchResult::SendBusy);
  EXPECT_EQ(fake.send_calls, 1);
}

TEST(SdkTransaction, RightOnlyDispatchReturnsOk)
{
  reset_fake();
  double left[7]{};
  double right[7]{};

  EXPECT_EQ(marvin_sdk_bridge::dispatch_position_commands(left, false, right, true),
            marvin_sdk_bridge::DispatchResult::Ok);
  EXPECT_EQ(fake.command_a_calls, 0);
  EXPECT_EQ(fake.command_b_calls, 1);
  EXPECT_EQ(fake.send_calls, 1);
}
