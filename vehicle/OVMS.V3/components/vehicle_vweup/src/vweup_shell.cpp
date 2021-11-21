/**
 * Project:      Open Vehicle Monitor System
 * Module:       VW e-Up via OBD Port
 *
 * (c) 2021 sharkcow <sharkcow@gmx.de>, Chris van der Meijden, SokoFromNZ, Michael Balzer <dexter@dexters-web.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// #include "ovms_log.h"
// static const char *TAG = "v-vweup";

#include <stdio.h>
#include <string>
#include <algorithm>
#include "ovms_command.h"
#include "ovms_utils.h"
#include "vehicle_vweup.h"
#include "vweup_obd.h"

using namespace std;


void OvmsVehicleVWeUp::ShellPollControl(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
{
  OvmsVehicleVWeUp* me = GetInstance(writer);
  string command = cmd->GetName();

  if (!me) {
    writer->puts("ERROR: vehicle module VWEUP not loaded");
    return;
  }
  else if (!me->HasOBD()) {
    writer->puts("ERROR: VWEUP OBD connection not enabled");
    return;
  }

  // change OBD state?
  obd_state_t state = me->m_obd_state;
  if (command == "pause")
    state = OBDS_Pause;
  else if (command == "continue")
    state = OBDS_Run;
  if (state != me->m_obd_state && !me->OBDSetState(state)) {
    writer->printf("ERROR: %s failed\n", command.c_str());
  }

  // report current/new status:
  writer->printf(
    "OBD polling status: %s\n"
    "OBD poller state: %s\n",
    GetOBDStateName(me->m_obd_state),
    GetPollStateName(me->m_poll_state));
}

void OvmsVehicleVWeUp::CCanData(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
{
  /* Buffer for CAN data */
  uint8_t data[10];

  /* This is an eUp, get the correct device instance... */
  OvmsVehicleVWeUp* me = GetInstance(writer);
  
  /* Access to comfort CAN */
  canbus *comf_bus;
  comf_bus = me->m_can3;

  writer->printf("CCanData: %i args\n", argc);

 

  /* Args are always pairs (ID + data) - i.e. allow only even number of args */
  if ( 0 != (argc % 2)) 
  {
    writer->printf("Invalid number of args!\n");
    return;
  }

  /* Go through all pairs and build a CAN message from each */
  for(int i=0; i<argc; i += 2)
  {
    const char* can_id_str = argv[i];
    const char* can_id_data_str = argv[i+1];
    //writer->printf("%i: %s\n", i, argv[i]);

    /* Get the CAN ID from the first arg string */
    char* endp;
    uint32_t can_id = strtoul(can_id_str, &endp, 16);

    if (*endp != '\0')
    {
      writer->printf("Invalid CAN ID in arg %i", i);
      return;
    }

    /* Ensure the data string has an even number of chars and does not exceed size of data */
    size_t hexlen = strlen(can_id_data_str);
    if ( (0U != (hexlen % 2U)) && ( (hexlen/2U) < sizeof(data)) )
    {
      writer->printf("Invalid number of characters in CAN data %i", i+1);
      return;
    }
    
    /* Go through data string and extract each byte (2-char hex ASCII) */
    uint8_t pos = 0U;
    for (size_t j=0U; j<hexlen; j += 2U) {
      const char* digit = &can_id_data_str[j];
      int ret = sscanf(digit, "%2hhx", &data[pos]);
      if (1 != ret) 
      {
        writer->printf("Hex character conversion failed at arg %i:%i", i, j);
        return; 
      }
      pos ++;
    }

    if (i == 0)
    {
      /* on first message, run wakeup first */
      (void)me->CommandWakeup();
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    /* All data extracted, send CAN message */
    writer->printf("CAN msg to %x: ", can_id);
    for (int k=0; k<hexlen/2; k++)
    {
      writer->printf(" %02x", data[k]);
    }
    writer->printf("\n");
    if (me->vweup_enable_write && !me->dev_mode) 
    {
      comf_bus->WriteStandard((uint16_t)can_id, hexlen/2U, data);
    }
  }


}