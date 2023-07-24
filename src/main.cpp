/*
 * Copyright (c) 2022 Mr. Green's Workshop https://www.MrGreensWorkshop.com
 *
 * SPDX-License-Identifier: Apache-2.0
 */
//
// Copyright 2023 Suzuki Yoshinori(wave.suzuki.z@gmail.com)
//
#include <array>
#include <btstack.h>
#include <btstack_run_loop.h>
#include <hardware/gpio.h>
#include <pico/cyw43_arch.h>
#include <pico/stdlib.h>

#define RFCOMM_SERVER_CHANNEL 1
#define HEARTBEAT_PERIOD_MS 1000

namespace
{
uint16_t                               rfcomm_channel_id;
std::array<uint8_t, 150>               spp_service_buffer;
btstack_timer_source_t                 heartbeat;
btstack_packet_callback_registration_t hci_event_callback_registration;
std::array<char, 30>                   lineBuffer;

//
void
packetHandler(uint8_t packetType, uint16_t channel, uint8_t* packet, uint16_t size)
{
  bd_addr_t event_addr;
  uint8_t   rfcomm_channel_nr;
  uint16_t  mtu;
  int       i;

  switch (packetType)
  {
  case HCI_EVENT_PACKET:
    switch (hci_event_packet_get_type(packet))
    {
    case HCI_EVENT_PIN_CODE_REQUEST:
      hci_event_pin_code_request_get_bd_addr(packet, event_addr);
      gap_pin_code_response(event_addr, "0000");
      break;

    case HCI_EVENT_USER_CONFIRMATION_REQUEST:
      break;

    case RFCOMM_EVENT_INCOMING_CONNECTION:
      rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
      rfcomm_channel_nr = rfcomm_event_incoming_connection_get_server_channel(packet);
      rfcomm_channel_id = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
      //   printf("RFCOMM channel %u requested for %s\n", rfcomm_channel_nr, bd_addr_to_str(event_addr));
      rfcomm_accept_connection(rfcomm_channel_id);
      break;

    case RFCOMM_EVENT_CHANNEL_OPENED:
      break;

    case RFCOMM_EVENT_CAN_SEND_NOW:
      rfcomm_send(rfcomm_channel_id, (uint8_t*)lineBuffer.data(), lineBuffer.size());
      break;

    case RFCOMM_EVENT_CHANNEL_CLOSED:
      rfcomm_channel_id = 0;
      break;

    default:
      break;
    }
    break;

  case RFCOMM_DATA_PACKET:

  {
    int accel = 0;
    int steer = 0;

    for (i = 0; i < size; i++)
    {
      int p = packet[i];
      if (p & 0x10)
        steer = p & 0xf;
      else
        accel = p & 0xf;
    }

    gpio_put(2, (accel & 0x3) ? 1 : 0);
    gpio_put(5, (accel & 0xc) ? 1 : 0);
    gpio_put(6, (steer & 0x3) ? 1 : 0);
    gpio_put(9, (steer & 0xc) ? 1 : 0);
    {
      static int n = 0;
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, n & 1);
      n++;
    }
  }
  break;

  default:
    break;
  }
}

void
spp_service_setup()
{
  hci_event_callback_registration.callback = &packetHandler;
  hci_add_event_handler(&hci_event_callback_registration);

  l2cap_init();

#ifdef ENABLE_BLE
  // Initialize LE Security Manager. Needed for cross-transport key derivation
  sm_init();
#endif

  rfcomm_init();
  rfcomm_register_service(packetHandler, RFCOMM_SERVER_CHANNEL, 0xffff); // reserved channel, mtu limited by l2cap

  // init SDP, create record for SPP and register with SDP
  sdp_init();
  spp_service_buffer.fill(0);
  spp_create_sdp_record(spp_service_buffer.data(), 0x10001, RFCOMM_SERVER_CHANNEL, "SPP Counter");
  sdp_register_service(spp_service_buffer.data());
  //   printf("SDP service record size: %u\n", de_get_len(spp_service_buffer));
}

void
heartbeatHandler(struct btstack_timer_source* ts)
{
  static int counter = 0;

  if (rfcomm_channel_id)
  {
    snprintf(lineBuffer.data(), lineBuffer.size(), "BTCNT %04u\n", ++counter);
    // printf("%s", lineBuffer);

    rfcomm_request_can_send_now_event(rfcomm_channel_id);
  }

  btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
  btstack_run_loop_add_timer(ts);
}

static void
one_shot_timer_setup()
{
  // set one-shot timer
  heartbeat.process = &heartbeatHandler;
  btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS);
  btstack_run_loop_add_timer(&heartbeat);
}

} // namespace

//
//
//
int
main()
{
  stdio_init_all();

  if (cyw43_arch_init())
  {
    // printf("cyw43_arch_init() failed.\n");
    return -1;
  }

  gpio_init(2);
  gpio_init(5);
  gpio_init(6);
  gpio_init(9);

  gpio_set_dir(2, GPIO_OUT);
  gpio_set_dir(5, GPIO_OUT);
  gpio_set_dir(6, GPIO_OUT);
  gpio_set_dir(9, GPIO_OUT);

  one_shot_timer_setup();
  spp_service_setup();

  // run the app
  gap_discoverable_control(1);
  gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
  gap_set_local_name("Pajero 00:00:00:00:00:00");

  // turn on!
  hci_power_control(HCI_POWER_ON);

  btstack_run_loop_execute();
}
